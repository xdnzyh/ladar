using System;
using System.IO;
using System.IO.Ports;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading;
using System.Diagnostics;
using System.Collections.Generic;
using System.Globalization;

interface ICarLink {
    long Now { get; }
    string Read();
    void Write(string s);
    void Sleep(int ms);
}
class SerialCarLink : ICarLink, IDisposable {
    readonly SerialPort port;
    readonly Stopwatch clock=Stopwatch.StartNew();
    public SerialCarLink(string name) {
        port=new SerialPort(name,9600,Parity.None,8,StopBits.One);
        port.DtrEnable=false; port.RtsEnable=false; port.Handshake=Handshake.None;
        // Single-byte mapping preserves FF/other invalid ASCII values for diagnostics.
        port.Encoding=Encoding.GetEncoding(28591); port.ReadTimeout=200; port.WriteTimeout=1500;
        port.Open();
    }
    public long Now {get{return clock.ElapsedMilliseconds;}}
    public string Read(){return port.ReadExisting();}
    public void Write(string s){port.Write(s);}
    public void Sleep(int ms){Thread.Sleep(ms);}
    public void Dispose(){port.Dispose();}
}
class CarSession {
    readonly ICarLink link;
    readonly Action<string> note;
    readonly Action<string> raw;
    readonly Func<string> newTag;
    string pending="";
    long lastReceive;
    public bool Uncertain {get; private set;}
    public int Padding {get; set;}
    public Func<bool> EmergencyRequested;
    public Func<bool> BeforeMove;
    public int ConfigurationEpoch {get; private set;}
    bool emergencyLatched;
    public int LastPingAttempts {get; private set;}
    public long LastPingMs {get; private set;}
    public string LastPingTag {get; private set;}
    public string LastResultSummary {get; private set;}
    string preparedTag;
    bool preparing, preparedPong, preparationFailed, preparationAnnounced;
    int preparedEpoch;
    long preparationSent;
    void ClearPreparation() {
        preparedTag=null;preparing=false;preparedPong=false;
        preparationFailed=false;preparationAnnounced=false;
    }
    // UI idle service only: never called from config exchanges or motion polling.
    public void PrepareNextMove() {
        if(Uncertain||emergencyLatched||(BeforeMove!=null&&!BeforeMove())) {ClearPreparation();return;}
        if(preparedTag!=null) {
            if(preparedEpoch!=ConfigurationEpoch){ClearPreparation();return;}
            if(preparing&&link.Now-preparationSent>=1800) {
                ClearPreparation();preparationFailed=true;
                note("空闲握手未完成，下一次运动时重新确认。");
            } else if(preparedPong&&!preparationAnnounced&&link.Now-lastReceive>=250) {
                preparationAnnounced=true;note("空闲握手已准备，下一条动作可直接发送。");
            }
            return;
        }
        if(preparationFailed||link.Now-lastReceive<500)return;
        // An old unterminated diagnostic must not swallow the preflight reply.
        if(pending.Length>0){note("空闲接收残片："+pending);pending="";}
        string tag=newTag();
        if(tag==null||!Regex.IsMatch(tag,@"\A[0-9A-F]{8}\z"))throw new ArgumentException("PING tag must be 8 uppercase hex digits.");
        preparedTag=tag;preparedEpoch=ConfigurationEpoch;
        preparationSent=link.Now;preparing=true;preparedPong=false;
        try {Send("@PING,"+tag);} catch {ClearPreparation();preparationFailed=true;throw;}
    }
    void WaitPreparation() {
        while(preparedTag!=null&&preparing&&link.Now-preparationSent<1800) {
            Poll();
            if(emergencyLatched||Uncertain){ClearPreparation();return;}
            if(preparing)link.Sleep(10);
        }
        if(preparing){ClearPreparation();preparationFailed=true;}
    }
    public void SuspendPreparation() {
        // A sent PING cannot be cancelled on the wire. Drain it before another
        // command/exchange starts, so a late PONG cannot overwrite a newer arm.
        WaitPreparation();ClearPreparation();
    }
    bool PrepareMotionHandshake() {
        WaitPreparation();
        if(preparedTag!=null&&preparedPong&&preparedEpoch==ConfigurationEpoch) {
            string tag=preparedTag;
            if(!Quiet(250,4000)||preparedTag!=tag||preparedEpoch!=ConfigurationEpoch)return false;
            LastPingTag=tag;
            note("沿用空闲时完成的握手，直接发送本次动作。");
            return true;
        }
        return Ping(5)&&Quiet(250,4000)&&LastPingTag!=null;
    }
    public CarSession(ICarLink l,Action<string> n,Action<string> r,Func<string> tag) {
        link=l; note=n; raw=r; newTag=tag; lastReceive=link.Now;
    }
    public List<string> Poll() {return Poll(false);}
    List<string> Poll(bool requireNewline) {
        if(!emergencyLatched && EmergencyRequested!=null && EmergencyRequested()) {
            emergencyLatched=true;Uncertain=true;
            ClearPreparation();
            note("已请求紧急停止，等待回报；不会继续发送MOVE。");
            link.Write("!\r\nX\r\n");
        }
        string chunk=link.Read();
        if(chunk.Length>0){lastReceive=link.Now; raw(chunk); pending+=chunk;}
        var lines=new List<string>(); int p;
        // Accept CR, LF and CRLF, including a CRLF pair split across reads.
        // Protected RESULT requires LF; legacy consumers retain CR-only compatibility.
        while((p=requireNewline?pending.IndexOf('\n'):pending.IndexOfAny(new[]{'\r','\n'}))>=0) {
            string line=pending.Substring(0,p); pending=pending.Substring(p+1);
            if(line.EndsWith("\r",StringComparison.Ordinal))line=line.Substring(0,line.Length-1);
            if(line.Length==0)continue;
            // Recover a full frame after a diagnostic whose line ending was lost.
            // Keep the physical RX line in the log; never invent a missing terminator.
            int frameStart=line.IndexOf('@');
            if(frameStart>0) {
                note("RX "+line);
                string prefix=line.Substring(0,frameStart);
                if(prefix.StartsWith("MECANUM UNIVERSAL",StringComparison.Ordinal)) {
                    ConfigurationEpoch++;
                    LastPingTag=null;
                    ClearPreparation();
                    note("检测到底盘启动，已确认的参数状态失效，请重新READ/APPLY。");
                }
                lines.Add(prefix);
                line=line.Substring(frameStart);
                note("从诊断残片后识别协议帧，仍须通过完整校验。");
            }
            if(line.StartsWith("MECANUM UNIVERSAL",StringComparison.Ordinal)) {
                ConfigurationEpoch++;
                LastPingTag=null;
                ClearPreparation();
                note("检测到底盘启动，已确认的参数状态失效，请重新READ/APPLY。");
            }
            if(line.StartsWith("FULL RESET",StringComparison.Ordinal)) {ClearPreparation();LastPingTag=null;}
            if(preparing&&preparedTag!=null&&preparedEpoch==ConfigurationEpoch) {
                if(line=="@PONG,"+preparedTag) {
                    preparing=false;preparedPong=true;LastPingTag=preparedTag;
                } else if(line=="@ERR,BUSY") {ClearPreparation();preparationFailed=true;}
            }
            lines.Add(line); note("RX "+line);
        }
        if(pending.Length>4096){Uncertain=true;ClearPreparation(); note("接收行过长，结果不确定。"); pending="";}
        return lines;
    }
    // Wait for actual RX silence. A fixed delay during an active report is insufficient.
    public bool Quiet(int silence,int maximum) {
        long start=link.Now;
        while(link.Now-start<maximum) {
            Poll();
            if(emergencyLatched)return false;
            if(link.Now-lastReceive>=silence) {
                if(pending.Length>0) {note("不完整接收尾部（未当作完成）："+pending); pending="";}
                return true;
            }
            link.Sleep(10);
        }
        note("接收一直未空闲，本次不发送。"); return false;
    }
    void Send(string command) {
        note("TX "+command+" [前导空格="+Padding+"]");
        link.Write(new string(' ',Padding)+command+"\r\n");
    }
    public bool Ping(int maximumAttempts) {
        SuspendPreparation();
        LastPingAttempts=0; LastPingMs=0; LastPingTag=null;
        if(!Quiet(500,4000))return false;
        for(int i=0;i<maximumAttempts;i++) {
            string tag=newTag(); LastPingAttempts++;
            if(tag==null||!Regex.IsMatch(tag,@"\A[0-9A-F]{8}\z"))throw new ArgumentException("PING tag must be 8 uppercase hex digits.");
            Send("@PING,"+tag); long start=link.Now;
            while(link.Now-start<1800) {
                var replies=Poll();
                if(emergencyLatched)return false;
                foreach(string line in replies) {
                    if(line=="@PONG,"+tag){
                        LastPingMs=link.Now-start;LastPingTag=tag;
                        preparedTag=tag;preparedEpoch=ConfigurationEpoch;preparedPong=true;
                        return true;
                    }
                    if(line=="@ERR,BUSY"){note("小车忙，没有发送移动。");return false;}
                }
                if(emergencyLatched)return false;
                link.Sleep(10);
            }
            if(!Quiet(500,4000))return false;
        }
        note("握手失败，没有发送移动。");return false;
    }
    public static string DoneReason(string line,string mode,string counts,string unit="CNT") {
        string target=unit=="MM"?",TARGET_CNT=[0-9]+":"";
        var m=Regex.Match(line,"^@DONE,"+Regex.Escape(mode)+",(TARGET|EMERGENCY|TIMEOUT|WRONG_DIRECTION),REQ="+Regex.Escape(counts)+",UNIT="+Regex.Escape(unit)+target+",BRAKE=-?[0-9]+\\.[0-9]+,ENC=-?[0-9]+\\.[0-9]+,DX=-?[0-9]+\\.[0-9]+,DY=-?[0-9]+\\.[0-9]+,DR=-?[0-9]+\\.[0-9]+,DS=-?[0-9]+\\.[0-9]+,Q1=-?[0-9]+,Q2=-?[0-9]+,Q3=-?[0-9]+,Q4=-?[0-9]+$");
        return m.Success?m.Groups[1].Value:null;
    }
    public void Confirm(){SuspendPreparation();Uncertain=false;emergencyLatched=false;note("已按现场确认解锁；上一动作不会补发。");}
    // CRC-16/CCITT-FALSE, ASCII body including '@', excluding the final comma/CRC.
    // Keep this independent of the config client for the standalone legacy sender build.
    public static string CheckedCommand(string body) {
        ushort crc=0xFFFF;
        foreach(char c in body) {
            if(c<32||c>126)throw new ArgumentException("Protocol body must be printable ASCII.");
            crc^=(ushort)(c<<8);
            for(int i=0;i<8;i++)crc=(ushort)((crc&0x8000)!=0?(crc<<1)^0x1021:crc<<1);
        }
        return body+","+crc.ToString("X4",CultureInfo.InvariantCulture);
    }
    static string[] CheckedFields(string line) {
        int cut=line.LastIndexOf(',');
        if(cut<0||line.Length-cut!=5)return null;
        foreach(char c in line)if(c<32||c>126)return null;
        string body=line.Substring(0,cut);
        if(!string.Equals(CheckedCommand(body),line,StringComparison.Ordinal))return null;
        return body.Split(',');
    }
    static int[] Directions(string mode) {
        switch(mode) {
            case "W":return new[]{1,1,1,1};
            case "S":return new[]{-1,-1,-1,-1};
            case "A":return new[]{1,-1,1,-1};
            case "D":return new[]{-1,1,-1,1};
            case "Q":return new[]{1,0,1,0};
            case "E":return new[]{0,1,0,1};
            case "Z":return new[]{0,-1,0,-1};
            case "C":return new[]{-1,0,-1,0};
            default:return null;
        }
    }
    static bool RequestValue(string value,out int number) {
        number=0;
        return value!=null&&Regex.IsMatch(value,@"\A[1-9][0-9]{0,9}\z")
            &&int.TryParse(value,NumberStyles.None,CultureInfo.InvariantCulture,out number)&&number>0;
    }
    static bool ResultFloat(string value,out double number) {
        number=0;
        // AVR long encoder deltas / float means; no exponent, whitespace, NaN or Infinity.
        return Regex.IsMatch(value,@"\A-?[0-9]{1,10}\.[0-9]{2}\z")
            &&double.TryParse(value,NumberStyles.AllowLeadingSign|NumberStyles.AllowDecimalPoint,CultureInfo.InvariantCulture,out number)
            &&number>=int.MinValue&&number<=2147483648.0;
    }
    static string ResultReason(string[] f,string id,string mode,string value,string unit,out string summary) {
        summary=null;
        if(f==null||f.Length!=12||f[0]!="@RESULT"||f[1]!=id||f[2]!=mode||f[4]!=value||f[5]!=unit)return null;
        int request;
        if(!RequestValue(f[4],out request)||f[3].Length!=1||f[3][0]<'0'||f[3][0]>'3')return null;
        double brake,enc;
        if(!ResultFloat(f[6],out brake)||!ResultFloat(f[7],out enc))return null;
        int[] directions=Directions(mode);if(directions==null)return null;
        float sum=0;int active=0;
        for(int i=0;i<4;i++) {
            int q;
            if(!Regex.IsMatch(f[8+i],@"\A-?[0-9]{1,10}\z")||!int.TryParse(f[8+i],NumberStyles.AllowLeadingSign,CultureInfo.InvariantCulture,out q))return null;
            if(directions[i]!=0){sum+=(float)((long)directions[i]*q);active++;}
        }
        // Match updateSegmentState's ordered float accumulation over active wheels.
        if(Math.Abs(enc-(double)(sum/active))>0.011)return null;
        string reason=new[]{"TARGET","EMERGENCY","TIMEOUT","WRONG_DIRECTION"}[f[3][0]-'0'];
        // RESULT lacks MM's TARGET_CNT; publish an explicit normalized summary, not a synthetic DONE.
        summary="RESULT_OK,ID="+id+",MODE="+mode+",REASON="+reason+",REQ="+value+",UNIT="+unit
            +",BRAKE_CNT="+f[6]+",ENC_CNT="+f[7]+",Q1="+f[8]+",Q2="+f[9]+",Q3="+f[10]+",Q4="+f[11];
        return reason;
    }
    public string MoveRecoverable(string mode,string counts,string unit="CNT",int timeoutMs=14000) {
        // Recovery never extends the original motion deadline.
        if(timeoutMs<14000||timeoutMs>21000)throw new ArgumentOutOfRangeException("timeoutMs");
        int request;
        if(Directions(mode)==null||!RequestValue(counts,out request)||(unit!="CNT"&&unit!="MM"))throw new ArgumentException("Invalid MOVE mode/value/unit.");
        if(Uncertain){note("上一动作结果待确认，禁止继续移动。");return "LOCKED";}
        if(BeforeMove!=null&&!BeforeMove()){ClearPreparation();note("参数尚未确认，请先READ/APPLY一次。");return "CONFIG_REQUIRED";}
        if(!PrepareMotionHandshake()||Uncertain||LastPingTag==null)return "NO_LINK";
        if(BeforeMove!=null&&!BeforeMove()){note("等待期间参数确认已失效，没有发送MOVE。");return "CONFIG_REQUIRED";}
        string id=LastPingTag;int moveEpoch=ConfigurationEpoch;
        ClearPreparation(); // This arm is consumed before any MOVE bytes can be written.
        LastResultSummary=null;
        Uncertain=true; // Latch before write: a partial write must never be replayed.
        long start=link.Now;
        Send(CheckedCommand("@MOVE,"+mode+","+counts+","+unit+","+id));
        int queries=0;long nextQuery=start+1800;bool recoveryCue=false;
        while(link.Now-start<timeoutMs) {
            long previousReceive=lastReceive;
            var lines=Poll(true);
            if(ConfigurationEpoch!=moveEpoch){note("运动期间检测到重启，结果不确定，不继续发送。");return "RESTART";}
            if(link.Now-start>=timeoutMs)break;
            foreach(string line in lines) {
                string summary;
                var f=CheckedFields(line);
                string reason=ResultReason(f,id,mode,counts,unit,out summary);
                if(reason!=null) {
                    LastResultSummary=summary;note(summary);
                    if(reason=="TARGET"&&!emergencyLatched){Uncertain=false;note("完成本次动作，结果校验通过。");}
                    else note("已收到校验终态，原因="+reason+"；停止请求或异常保持锁定，请现场确认后解锁。");
                    return reason;
                }
                // ACK and unchecked DONE never establish protected completion. Corruption
                // of a report's prefix still triggers recovery after actual RX silence.
                if(line!="@ACK,"+mode+","+counts+","+unit)recoveryCue=true;
                if(f!=null&&f.Length==3&&f[0]=="@RESULT"&&f[1]==id&&(f[2]=="B"||f[2]=="N")) {
                    note(f[2]=="B"?"动作仍在运行，等待校验结果。":"设备尚无该动作快照，继续只读查询；不补发MOVE。");
                    if(f[2]=="B")nextQuery=Math.Max(nextQuery,link.Now+1800);
                }
            }
            if(pending.Length>0||(lastReceive!=previousReceive&&lines.Count==0))recoveryCue=true;
            long now=link.Now;
            // Preserve a final retrieval after a long-running move has settled, even
            // when all earlier reports (including B) were lost or corrupted.
            if(queries==4)nextQuery=Math.Max(nextQuery,start+timeoutMs-1500);
            if(queries<5 && now-lastReceive>=250 && (queries==0?(recoveryCue||now>=nextQuery):now>=nextQuery)) {
                // Discard only an unterminated damaged tail, after quiet, before querying.
                if(pending.Length>0){note("不完整结果尾部（未当作完成）："+pending);pending="";}
                nextQuery=now+1200;queries++;
                Send(CheckedCommand("@RESULT,"+id));
            }
            link.Sleep(10);
        }
        note(timeoutMs+"ms内未收到匹配且校验完整的RESULT，结果不确定。MOVE未重发；请现场确认后CONFIRM。");
        return "UNCERTAIN";
    }
    public string Move(string mode,string counts,string unit="CNT",int timeoutMs=14000) {
        if(timeoutMs<14000||timeoutMs>21000)throw new ArgumentOutOfRangeException("timeoutMs");
        if(Uncertain){note("上一动作结果待确认，禁止继续移动。");return "LOCKED";}
        if(BeforeMove!=null&&!BeforeMove()){note("参数尚未确认，请先READ/APPLY一次。");return "CONFIG_REQUIRED";}
        if(!Ping(5))return "NO_LINK";
        if(!Quiet(250,4000) || Uncertain)return "NO_LINK";
        if(BeforeMove!=null&&!BeforeMove()){note("等待期间参数确认已失效，没有发送MOVE。");return "CONFIG_REQUIRED";}
        int moveEpoch=ConfigurationEpoch;
        ClearPreparation();
        Uncertain=true; // A write error may occur after some or all bytes were sent.
        Send("@MOVE,"+mode+","+counts+","+unit); // Only one transmission, never retried.
        bool ack=false;long start=link.Now;
        while(link.Now-start<timeoutMs) {
            foreach(string line in Poll()) {
                if(ConfigurationEpoch!=moveEpoch){note("运动期间检测到重启，结果不确定，不继续发送。");return "RESTART";}
                if(line=="@ACK,"+mode+","+counts+","+unit)ack=true;
                else if(line.StartsWith("@ACK,",StringComparison.Ordinal)) {
                    note("ACK与请求不一致：停止继续发送，等待现场确认。");return "MISMATCH";
                }
                string reason=DoneReason(line,mode,counts,unit);
                if(reason!=null) {
                    if(reason=="TARGET" && !emergencyLatched) {
                        Uncertain=false;
                        note(ack?"本次到达目标，完整DONE已保存。":"收到完整TARGET DONE，ACK缺失；本次已执行，不重发。");
                    } else note("已收到终态，原因="+reason+"；停止请求或异常保持锁定，请现场确认后解锁。");
                    return reason;
                }
            }
            link.Sleep(10);
        }
        note("未收到完整DONE，结果不确定。没有自动重发；先检查车辆并记录本次，再CONFIRM解锁。");return "UNCERTAIN";
    }
    public void Check(int count) {
        int first=0,eventual=0,sends=0;long elapsed=0;
        note("开始只读通信检查，共"+count+"轮。不会发送MOVE。");
        for(int i=0;i<count;i++) {
            bool ok=Ping(5);sends+=LastPingAttempts;
            if(ok){eventual++;elapsed+=LastPingMs;if(LastPingAttempts==1)first++;}
            note("CHECK "+(i+1)+"/"+count+" "+(ok?"成功":"失败")+" 发送次数="+LastPingAttempts+" 响应ms="+LastPingMs);
        }
        note("CHECK汇总：首发成功="+first+"/"+count+"，重试后成功="+eventual+"/"+count+"，PING总发送="+sends+"，成功应答平均ms="+(eventual==0?0:elapsed/eventual)+"，前导空格="+Padding);
    }
}
