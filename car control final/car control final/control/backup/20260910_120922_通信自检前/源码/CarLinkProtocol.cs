using System;
using System.IO;
using System.IO.Ports;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading;
using System.Diagnostics;
using System.Collections.Generic;

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
    public CarSession(ICarLink l,Action<string> n,Action<string> r,Func<string> tag) {
        link=l; note=n; raw=r; newTag=tag; lastReceive=link.Now;
    }
    public List<string> Poll() {
        if(!emergencyLatched && EmergencyRequested!=null && EmergencyRequested()) {
            emergencyLatched=true;Uncertain=true;
            note("已请求紧急停止，等待回报；不会继续发送MOVE。");
            link.Write("!\r\nX\r\n");
        }
        string chunk=link.Read();
        if(chunk.Length>0){lastReceive=link.Now; raw(chunk); pending+=chunk;}
        var lines=new List<string>(); int p;
        // Accept CR, LF and CRLF, including a CRLF pair split across reads.
        while((p=pending.IndexOfAny(new[]{'\r','\n'}))>=0) {
            string line=pending.Substring(0,p).TrimEnd('\r'); pending=pending.Substring(p+1);
            if(line.Length==0)continue;
            if(line.StartsWith("MECANUM UNIVERSAL",StringComparison.Ordinal)) {
                ConfigurationEpoch++;
                note("检测到底盘启动，已确认的参数状态失效，请重新READ/APPLY。");
            }
            lines.Add(line); note("RX "+line);
        }
        if(pending.Length>4096){Uncertain=true; note("接收行过长，结果不确定。"); pending="";}
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
        LastPingAttempts=0; LastPingMs=0;
        if(!Quiet(500,4000))return false;
        for(int i=0;i<maximumAttempts;i++) {
            string tag=newTag(); LastPingAttempts++;
            Send("@PING,"+tag); long start=link.Now;
            while(link.Now-start<1800) {
                foreach(string line in Poll()) {
                    if(line=="@PONG,"+tag){LastPingMs=link.Now-start;return true;}
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
    public void Confirm(){Uncertain=false;emergencyLatched=false;note("已按现场确认解锁；上一动作不会补发。");}
    public string Move(string mode,string counts,string unit="CNT",int timeoutMs=14000) {
        if(timeoutMs<14000||timeoutMs>21000)throw new ArgumentOutOfRangeException("timeoutMs");
        if(Uncertain){note("上一动作结果待确认，禁止继续移动。");return "LOCKED";}
        if(BeforeMove!=null&&!BeforeMove()){note("参数尚未确认，请先READ/APPLY一次。");return "CONFIG_REQUIRED";}
        if(!Ping(5))return "NO_LINK";
        if(!Quiet(250,4000) || Uncertain)return "NO_LINK";
        if(BeforeMove!=null&&!BeforeMove()){note("等待期间参数确认已失效，没有发送MOVE。");return "CONFIG_REQUIRED";}
        int moveEpoch=ConfigurationEpoch;
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
