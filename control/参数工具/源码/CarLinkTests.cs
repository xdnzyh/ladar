using System;
using System.IO;
using System.Text;
using System.Collections.Generic;

class FakeCarLink : ICarLink {
    public long Now {get;private set;}
    public readonly List<string> Writes=new List<string>();
    public readonly List<long> WriteTimes=new List<long>();
    readonly SortedDictionary<long,List<string>> incoming=new SortedDictionary<long,List<string>>();
    public Action<string,FakeCarLink> OnWrite;
    public void Later(int ms,string s){long t=Now+ms;if(!incoming.ContainsKey(t))incoming[t]=new List<string>();incoming[t].Add(s);}
    public string Read(){var s=new StringBuilder();var remove=new List<long>();foreach(var e in incoming){if(e.Key>Now)break;foreach(string v in e.Value)s.Append(v);remove.Add(e.Key);}foreach(long key in remove)incoming.Remove(key);return s.ToString();}
    public void Write(string s){Writes.Add(s);WriteTimes.Add(Now);if(OnWrite!=null)OnWrite(s,this);}
    public void Sleep(int ms){Now+=ms;}
}
class CarLinkTests {
    static readonly string Done="@DONE,Q,TARGET,REQ=1200,UNIT=CNT,BRAKE=1145.00,ENC=1203.00,DX=602.00,DY=601.00,DR=-1.00,DS=0.00,Q1=1204,Q2=0,Q3=1202,Q4=0";
    static void Assert(bool value){if(!value)throw new Exception("Sender test assertion failed");}
    const string ResultBody="@RESULT,1234ABCD,W,0,718,CNT,639.00,639.00,639,639,639,639";
    static string ResultFrame(string body){return CarSession.CheckedCommand(body)+"\r\n";}
    static int MoveCount(FakeCarLink link){return link.Writes.FindAll(s=>s.Contains("@MOVE,")).Count;}
    static List<long> QueryTimes(FakeCarLink link) {
        var times=new List<long>();
        for(int i=0;i<link.Writes.Count;i++)if(link.Writes[i].TrimStart().StartsWith("@RESULT,"))times.Add(link.WriteTimes[i]);
        return times;
    }
    static void RecoveryReplay(string report,int expectedQueries,bool recover=true,int finalQuery=1) {
        var link=new FakeCarLink();var notes=new List<string>();int queries=0;long moveAt=-1;
        var session=new CarSession(link,s=>notes.Add(s),s=>{},()=>"1234ABCD");
        session.Padding=32;
        link.OnWrite=(wire,f)=>{
            Assert(wire.StartsWith(new string(' ',32))&&wire.EndsWith("\r\n"));
            string s=wire.Substring(32);
            if(s=="@PING,1234ABCD\r\n"){f.Later(20,"@PONG,1234ABCD\r\n");return;}
            if(s.StartsWith("@MOVE,")) {
                Assert(s==ResultFrame("@MOVE,W,718,CNT,1234ABCD"));moveAt=f.Now;
                f.Later(20,"@ACK,W,718,CNT\r\n");
                if(report!=null)f.Later(100,report);
                return;
            }
            Assert(s=="@RESULT,1234ABCD,8CE5\r\n");queries++;
            if(recover)f.Later(20,queries>=finalQuery?ResultFrame(ResultBody):ResultFrame(ResultBody).Replace("639.00","638.00"));
        };
        string outcome=session.MoveRecoverable("W","718");
        Assert(outcome==(recover?"TARGET":"UNCERTAIN"));
        Assert(session.Uncertain==!recover&&MoveCount(link)==1&&queries==expectedQueries);
        Assert(session.LastPingTag=="1234ABCD"&&session.LastPingAttempts==1);
        Assert(link.Writes.FindAll(s=>s.Contains("@PING,")).Count==1);
        var times=QueryTimes(link);
        for(int i=1;i<times.Count;i++)Assert(times[i]-times[i-1]>=1200);
        if(times.Count==5)Assert(times[4]-moveAt==12500);
        if(times.Count>0)Assert(times[0]-moveAt==(report==null?1800:350));
        if(recover)Assert(session.LastResultSummary=="RESULT_OK,ID=1234ABCD,MODE=W,REASON=TARGET,REQ=718,UNIT=CNT,BRAKE_CNT=639.00,ENC_CNT=639.00,Q1=639,Q2=639,Q3=639,Q4=639");
        else {
            Assert(link.Now-moveAt==14000&&session.LastResultSummary==null);
            Assert(session.MoveRecoverable("W","718")=="LOCKED"&&MoveCount(link)==1);
        }
    }
    static void RecoverableResults() {
        Assert(CarSession.CheckedCommand("123456789")=="123456789,29B1");
        Assert(ResultFrame(ResultBody)=="@RESULT,1234ABCD,W,0,718,CNT,639.00,639.00,639,639,639,639,7EF0\r\n");
        Assert(CarSession.CheckedCommand("@RESULT,1234ABCD")=="@RESULT,1234ABCD,8CE5");
        string frame=ResultFrame(ResultBody);
        RecoveryReplay(frame,0); // Immediate auto RESULT: one PING, one MOVE, zero queries.
        RecoveryReplay(null,1);
        foreach(string damaged in new[]{frame.Substring(1),frame.Substring(16),frame.Replace("@RESULT","@RESULX"),
            frame.Substring(0,frame.Length-12)+"\r\n",frame.Substring(0,frame.Length-6)+"7EF1\r\n",
            frame.Replace("639.00","638.00"),frame.TrimEnd('\r','\n'),frame.TrimEnd('\n'),
            Done+"\r\n",Done.Substring(0,70),"FREE_RAM_START=84IT=CNT,BRAKE=639.00\r\n"})
            RecoveryReplay(damaged,1);
        // A single flipped ASCII bit remains a digit and even preserves Q/ENC consistency
        // when BRAKE changes; the old CRC must still reject it.
        var numericBit=frame.ToCharArray();numericBit[frame.IndexOf("639.00")+2]^=(char)1;
        Assert(numericBit[frame.IndexOf("639.00")+2]=='8');
        RecoveryReplay(new string(numericBit),1);
        RecoveryReplay(null,5,true,5); // Four corrupted query replies, fifth returns the same snapshot.
        RecoveryReplay(null,5,false);
        RecoveryReplay(Done.Replace(",Q,",",W,").Replace("REQ=1200","REQ=718")+"\r\n",5,false);
        foreach(string invalidBody in new[]{ResultBody.Replace("1234ABCD","FFFFFFFF"),ResultBody.Replace(",W,",",S,"),
            ResultBody.Replace(",718,",",719,"),ResultBody.Replace(",CNT,",",MM,"),ResultBody.Replace(",W,0,",",W,4,"),
            ResultBody.Replace("639.00,639.00","639.00,640.00"),ResultBody.Replace("639.00,639.00","NaN,639.00"),
            ResultBody.Replace("639.00,639.00","Infinity,639.00"),ResultBody.Replace("639.00,639.00","2147483649.00,639.00"),
            ResultBody.Replace("639.00,639.00","-2147483649.00,639.00"),ResultBody+",0",
            ResultBody.Substring(0,ResultBody.Length-4)+",2147483648",ResultBody.Substring(0,ResultBody.Length-4)+",-2147483649",
            ResultBody.Replace(",718,",",2147483648,"),ResultBody.Replace(",718,",",0,"),
            ResultBody.Replace("639.00,639.00","639,639.00"),ResultBody.Replace("639.00,639.00"," 639.00,639.00")})
            RecoveryReplay(ResultFrame(invalidBody),1);
        RecoveryReplay(ResultFrame(ResultBody.Replace("1234ABCD","FFFFFFFF")),5,false);
        // Every truncation of the actual report, including a complete CRC without LF,
        // must recover on the FIRST query rather than concatenate with its partial tail.
        for(int cut=1;cut<frame.Length;cut++)RecoveryReplay(frame.Substring(0,cut),1);
        for(int split=0;split<=frame.Length;split++) {
            var link=new FakeCarLink();long moveAt=-1;
            var session=new CarSession(link,s=>{},s=>{},()=>"1234ABCD");
            link.OnWrite=(s,f)=>{
                if(s.StartsWith("@PING,")){f.Later(20,"@PONG,1234ABCD\r\n");return;}
                Assert(s.StartsWith("@MOVE,"));moveAt=f.Now;
                f.Later(100,frame.Substring(0,split));f.Later(120,frame.Substring(split));
            };
            Assert(session.MoveRecoverable("W","718")=="TARGET"&&!session.Uncertain);
            Assert(QueryTimes(link).Count==0&&MoveCount(link)==1&&link.Now-moveAt<=120);
        }
        Console.WriteLine("PASS: RESULT CRC golden vectors, immediate auto result, all byte splits/truncations, numeric bit corruption, metadata/bounds/kinematics rejection, first-query partial-tail recovery, five retries, wrong ID, deadline, no MOVE replay.");
    }
    static void RecoverySafety() {
        foreach(int deadline in new[]{14000,21000})foreach(string early in new[]{"lost","busy","corrupt"}) {
            var link=new FakeCarLink();long moveAt=-1;int queries=0;
            var session=new CarSession(link,s=>{},s=>{},()=>"1234ABCD");
            link.OnWrite=(s,f)=>{
                if(s.StartsWith("@PING,")){f.Later(20,"@PONG,1234ABCD\r\n");return;}
                if(s.StartsWith("@MOVE,")){moveAt=f.Now;f.Later(20,"@ACK,W,718,CNT\r\n");return;}
                Assert(s=="@RESULT,1234ABCD,8CE5\r\n");queries++;
                // Motion completes at 8 s (14 s for an extended firmware timeout).
                // Its automatic result is lost; only the immutable snapshot remains.
                if(f.Now-moveAt>=(deadline==14000?8000:14000))f.Later(20,ResultFrame(ResultBody));
                else if(early!="lost")f.Later(20,early=="busy"?ResultFrame("@RESULT,1234ABCD,B"):"@RESULT,1234ABCD,B,0000\r\n");
            };
            Assert(session.MoveRecoverable("W","718","CNT",deadline)=="TARGET"&&!session.Uncertain);
            Assert(queries==5&&MoveCount(link)==1&&QueryTimes(link)[4]-moveAt==deadline-1500);
        }
        foreach(string scenario in new[]{"quiet","busy","absent","late","emergency","stoprace","reboot","rebootglued","writeerror","config","configwait","restartwait","pingretry"}) {
            var link=new FakeCarLink();long moveAt=-1,lastRx=-1;int queries=0,pings=0,checks=0;
            var session=new CarSession(link,s=>{},s=>lastRx=link.Now,()=>pings==0?"00000001":"1234ABCD");
            session.BeforeMove=()=>{checks++;return scenario!="config"&&(scenario!="configwait"||checks==1);};
            if(scenario=="emergency"||scenario=="stoprace")session.EmergencyRequested=()=>moveAt>=0&&link.Now-moveAt>=80;
            link.OnWrite=(s,f)=>{
                if(s.StartsWith("@PING,")) {
                    pings++;
                    if(scenario=="pingretry"&&pings==1)return;
                    f.Later(20,s.Replace("@PING,","@PONG,"));
                    if(scenario=="restartwait")f.Later(100,"MECANUM UNIVERSAL V6.3\r\n");
                    return;
                }
                string body=ResultBody.Replace("1234ABCD",session.LastPingTag??"00000001");
                if(s=="!\r\nX\r\n") {f.Later(20,ResultFrame(scenario=="emergency"?body.Replace(",W,0,",",W,1,"):body));return;}
                if(s.StartsWith("@MOVE,")) {
                    moveAt=f.Now;Assert(s==ResultFrame("@MOVE,W,718,CNT,"+session.LastPingTag));
                    if(scenario=="writeerror")throw new IOException("partial tagged MOVE write");
                    f.Later(20,"@ACK,W,718,CNT\r\n");
                    if(scenario=="reboot"||scenario=="rebootglued")f.Later(100,"MECANUM UNIVERSAL V6.3"+(scenario=="reboot"?"\r\n":"")+ResultFrame(body));
                    else if(scenario=="quiet")for(int i=100;i<=2100;i+=100)f.Later(i,i==100?"@RESUL":"x");
                    else if(scenario=="late")f.Later(13990,ResultFrame(body));
                    else if(scenario=="pingretry")f.Later(100,ResultFrame(body));
                    return;
                }
                Assert(s==ResultFrame("@RESULT,"+session.LastPingTag));queries++;
                Assert(f.Now-lastRx>=250);
                if(scenario=="quiet"){Assert(f.Now-moveAt==2350);f.Later(20,ResultFrame(body));}
                else if(scenario=="busy")f.Later(20,ResultFrame(queries<5?"@RESULT,"+session.LastPingTag+",B":body));
                else if(scenario=="absent")f.Later(20,ResultFrame("@RESULT,"+session.LastPingTag+",N"));
            };
            string result;
            try{result=session.MoveRecoverable("W","718");}catch(IOException){result="writeerror";}
            string expected=scenario=="config"||scenario=="configwait"?"CONFIG_REQUIRED":scenario=="restartwait"?"NO_LINK":
                scenario=="emergency"?"EMERGENCY":scenario=="reboot"||scenario=="rebootglued"?"RESTART":
                scenario=="writeerror"?"writeerror":scenario=="absent"?"UNCERTAIN":"TARGET";
            if(result!=expected)throw new Exception("Recovery scenario "+scenario+": "+result+" expected "+expected);
            Assert(MoveCount(link)==(moveAt<0?0:1));
            if(moveAt>=0)Assert(session.Uncertain==(expected!="TARGET"||scenario=="stoprace"));
            Assert(checks==(scenario=="config"||scenario=="restartwait"?1:2));
            if(scenario=="busy"||scenario=="absent")Assert(queries==5);
            if(scenario=="late")Assert(link.Now-moveAt==13990);
            if(scenario=="emergency"||scenario=="stoprace")Assert(link.Writes.FindAll(s=>s=="!\r\nX\r\n").Count==1);
            if(scenario=="reboot"||scenario=="rebootglued"||scenario=="restartwait")Assert(session.ConfigurationEpoch==1&&session.LastPingTag==null);
            if(scenario=="pingretry")Assert(pings==2&&session.LastPingTag=="1234ABCD"&&queries==0);
        }
        // Active directional means must ignore inactive wheel drift on diagonals.
        string[] modes={"W","S","A","D","Q","E","Z","C"};
        string[] wheels={"600,602,604,606","-600,-602,-604,-606","600,-602,604,-606","-600,602,-604,606",
            "600,77,606,-33","77,600,-33,606","77,-600,-33,-606","-600,77,-606,-33"};
        for(int i=0;i<modes.Length;i++)foreach(string unit in new[]{"CNT","MM"})foreach(int reason in new[]{0,1,2,3}) {
            var link=new FakeCarLink();var session=new CarSession(link,s=>{},s=>{},()=>"1234ABCD");
            string mode=modes[i];
            link.OnWrite=(s,f)=>{
                if(s.StartsWith("@PING,")){f.Later(20,"@PONG,1234ABCD\r\n");return;}
                Assert(s==ResultFrame("@MOVE,"+mode+",718,"+unit+",1234ABCD"));
                f.Later(100,ResultFrame("@RESULT,1234ABCD,"+mode+","+reason+",718,"+unit+",599.00,603.00,"+wheels[i]));
            };
            Assert(session.MoveRecoverable(mode,"718",unit)==new[]{"TARGET","EMERGENCY","TIMEOUT","WRONG_DIRECTION"}[reason]);
            Assert(session.Uncertain==(reason!=0)&&QueryTimes(link).Count==0);
        }
        Console.WriteLine("PASS: actual RX silence, B/N, late auto result within deadline, emergency/race, reboot epoch, write failure, cached BeforeMove, successful retry PING tag, all 8 directions/units/reasons.");
    }

    static void DiagnosticFragmentReplay() {
        string done="@DONE,W,TARGET,REQ=718,UNIT=CNT,BRAKE=634.50,ENC=739.25,DX=739.25,DY=-4.25,DR=-1.25,DS=-3.25,Q1=733,Q2=739,Q3=737,Q4=748";
        string wire="FREE_RAM_START=84"+done+"\r\n";
        for(int split=0;split<=wire.Length;split++) {
            var link=new FakeCarLink();var received=new StringBuilder();
            var session=new CarSession(link,s=>{},s=>received.Append(s),()=>"ABCDEF12");
            link.Later(0,wire.Substring(0,split));var lines=session.Poll();
            link.Later(0,wire.Substring(split));lines.AddRange(session.Poll());
            Assert(lines.Count==2 && lines[1]==done && received.ToString()==wire);
            Assert(CarSession.DoneReason(lines[1],"W","718")=="TARGET");
        }
        foreach(string kind in new[]{"valid","missingend","missingfields","missinghead","wrongrequest","damagedprotocol"}) {
            string reply=done;
            if(kind=="missingfields")reply=done.Substring(0,done.Length-4);
            if(kind=="missinghead")reply=done.Substring(20);
            if(kind=="wrongrequest")reply=done.Replace("REQ=718","REQ=719");
            if(kind=="damagedprotocol")reply="@DONE,W,TARGET,REQ="+done;
            string payload="FREE_RAM_START=84"+reply+(kind=="missingend"?"":"\r\n");
            var link=new FakeCarLink();var session=new CarSession(link,s=>{},s=>{},()=>"ABCDEF12");
            link.OnWrite=(s,f)=>{
                if(s.StartsWith("@PING,")){f.Later(20,s.Replace("@PING,","@PONG,"));return;}
                f.Later(20,"@ACK,W,718,CNT\r\n");
                f.Later(100,payload.Substring(0,30));f.Later(120,payload.Substring(30));
            };
            Assert((session.Move("W","718")=="TARGET")==(kind=="valid"));
            Assert(session.Uncertain==(kind!="valid"));
            Assert(link.Writes.FindAll(s=>s.Contains("@MOVE,")).Count==1);
        }
        Console.WriteLine("PASS: field diagnostic/DONE replay, all byte splits, incomplete frames rejected, one MOVE.");
    }
    static void PreflightAssert(bool value,string detail) {
        if(!value)throw new Exception(detail);
    }
    static int PingCount(FakeCarLink link) {return link.Writes.FindAll(s=>s.TrimStart().StartsWith("@PING,")).Count;}
    static void IdlePreparation(CarSession session,FakeCarLink link,int elapsed) {
        link.Sleep(elapsed);session.Poll();long before=link.Now;
        session.PrepareNextMove();
        PreflightAssert(link.Now==before,"PrepareNextMove blocked instead of returning at the same Now");
    }
    static CarSession PreflightSession(FakeCarLink link,int pongDelay,bool autoResult) {
        int nonce=0;
        var session=new CarSession(link,s=>{},s=>{},()=> (++nonce).ToString("X8"));
        link.OnWrite=(wire,f)=>{
            string s=wire.TrimStart();
            if(s.StartsWith("@PING,")) {
                if(pongDelay>=0)f.Later(pongDelay,s.Replace("@PING,","@PONG,"));
            } else if(s.StartsWith("@MOVE,")) {
                PreflightAssert(s==ResultFrame("@MOVE,W,718,CNT,"+session.LastPingTag),"MOVE did not use the confirmed tag");
                if(autoResult)f.Later(10,ResultFrame(ResultBody.Replace("1234ABCD",session.LastPingTag)));
            }
        };
        return session;
    }
    static long LastMoveTime(FakeCarLink link) {
        for(int i=link.Writes.Count-1;i>=0;i--)if(link.Writes[i].TrimStart().StartsWith("@MOVE,"))return link.WriteTimes[i];
        throw new Exception("Expected MOVE was not sent");
    }
    static void ReadyPreparation(CarSession session,FakeCarLink link) {
        IdlePreparation(session,link,500);
        IdlePreparation(session,link,20);
        IdlePreparation(session,link,250);
        PreflightAssert(PingCount(link)==1&&session.LastPingTag=="00000001","Poll did not confirm the background PONG");
    }
    static void IdlePreflight() {
        var cases=new Dictionary<string,Action>();
        cases.Add("actual RX silence / immediate prepared MOVE",()=>{
            var link=new FakeCarLink();var session=PreflightSession(link,20,true);
            IdlePreparation(session,link,499);
            PreflightAssert(link.Writes.Count==0,"PING before initial 500 ms silence");
            link.Later(0,"telemetry\r\n");IdlePreparation(session,link,0);
            IdlePreparation(session,link,499);
            PreflightAssert(link.Writes.Count==0,"PING before 500 ms since actual RX");
            IdlePreparation(session,link,1);
            PreflightAssert(PingCount(link)==1&&link.WriteTimes[0]==999,"First eligible idle call must send exactly one PING");
            IdlePreparation(session,link,20);
            PreflightAssert(session.LastPingTag=="00000001","Matching background PONG was not recognized by Poll");
            IdlePreparation(session,link,250);long requested=link.Now;
            PreflightAssert(session.MoveRecoverable("W","718")=="TARGET","Prepared MOVE failed");
            PreflightAssert(LastMoveTime(link)==requested&&PingCount(link)==1,"Prepared MOVE added a PING or a new silence wait");
        });
        cases.Add("prepared for twenty seconds / consumed once",()=>{
            var link=new FakeCarLink();var session=PreflightSession(link,20,true);
            ReadyPreparation(session,link);
            for(int i=0;i<2000;i++)IdlePreparation(session,link,10);
            PreflightAssert(PingCount(link)==1&&MoveCount(link)==0,"Idle preparation flooded PING or sent MOVE");
            long requested=link.Now;
            PreflightAssert(session.MoveRecoverable("W","718")=="TARGET"&&LastMoveTime(link)==requested,"Long-idle prepared MOVE lost readiness");
            PreflightAssert(session.MoveRecoverable("W","718")=="TARGET","Second foreground MOVE failed");
            PreflightAssert(PingCount(link)==2&&MoveCount(link)==2&&session.LastPingTag=="00000002","Second MOVE reused a consumed nonce");
            IdlePreparation(session,link,500);IdlePreparation(session,link,20);IdlePreparation(session,link,250);
            requested=link.Now;
            PreflightAssert(session.MoveRecoverable("W","718")=="TARGET"&&LastMoveTime(link)==requested,"Next idle preparation failed");
            PreflightAssert(PingCount(link)==3&&MoveCount(link)==3&&session.LastPingTag=="00000003","Sequential idle-prepared MOVE reused a nonce");
        });
        cases.Add("pending PONG adoption / remaining wait",()=>{
            var link=new FakeCarLink();var session=PreflightSession(link,1500,true);
            IdlePreparation(session,link,500);IdlePreparation(session,link,1000);
            long requested=link.Now;
            PreflightAssert(session.MoveRecoverable("W","718")=="TARGET","Pending preflight was not adopted");
            PreflightAssert(PingCount(link)==1&&LastMoveTime(link)==requested+750,"Adoption restarted the PING or its timeout/quiet interval");
        });
        cases.Add("PONG and subsequent RX require 250 ms quiet",()=>{
            foreach(bool telemetry in new[]{false,true}) {
                var link=new FakeCarLink();var session=PreflightSession(link,20,true);
                IdlePreparation(session,link,500);IdlePreparation(session,link,20);
                if(telemetry){link.Later(100,"telemetry\r\n");IdlePreparation(session,link,100);}
                long lastRx=link.Now;IdlePreparation(session,link,240);
                PreflightAssert(session.MoveRecoverable("W","718")=="TARGET","Quiet-boundary MOVE failed");
                PreflightAssert(PingCount(link)==1&&LastMoveTime(link)==lastRx+250,"MOVE ignored actual RX quiet or added a 500 ms wait");
            }
        });
        cases.Add("wrong and fragmented background PONG",()=>{
            var link=new FakeCarLink();var session=PreflightSession(link,-1,true);
            IdlePreparation(session,link,500);
            link.Later(20,"@PONG,FFFFFFFF\r\n");IdlePreparation(session,link,20);
            PreflightAssert(session.LastPingTag==null,"Mismatching PONG confirmed preparation");
            link.Later(20,"@PONG,0000");IdlePreparation(session,link,20);
            PreflightAssert(session.LastPingTag==null,"Partial PONG confirmed preparation");
            link.Later(20,"0001\r\n");IdlePreparation(session,link,20);IdlePreparation(session,link,250);
            long requested=link.Now;
            PreflightAssert(session.MoveRecoverable("W","718")=="TARGET"&&PingCount(link)==1&&LastMoveTime(link)==requested,"Fragmented matching PONG was not adopted");
        });
        cases.Add("startup invalidates pending and prepared tags",()=>{
            foreach(bool ready in new[]{false,true})foreach(bool glued in new[]{false,true}) {
                var link=new FakeCarLink();var session=PreflightSession(link,20,true);
                if(ready)ReadyPreparation(session,link);else IdlePreparation(session,link,500);
                link.Later(0,"MECANUM UNIVERSAL V6.3"+(glued?"":"\r\n")+"@PONG,00000001\r\n");session.Poll();
                PreflightAssert(session.ConfigurationEpoch==1&&session.LastPingTag==null,"Startup retained old confirmed tag");
                PreflightAssert(session.MoveRecoverable("W","718")=="TARGET"&&PingCount(link)==2&&session.LastPingTag=="00000002","MOVE reused pre-restart preparation");
            }
        });
        cases.Add("BeforeMove gate blocks idle and foreground sending",()=>{
            var link=new FakeCarLink();var session=PreflightSession(link,20,true);bool confirmed=false;
            session.BeforeMove=()=>confirmed;
            for(int i=0;i<200;i++)IdlePreparation(session,link,100);
            PreflightAssert(session.MoveRecoverable("W","718")=="CONFIG_REQUIRED"&&link.Writes.Count==0,"Unconfirmed configuration sent preflight or MOVE");
            confirmed=true;IdlePreparation(session,link,0);IdlePreparation(session,link,20);IdlePreparation(session,link,250);
            confirmed=false;int writes=link.Writes.Count;
            PreflightAssert(session.MoveRecoverable("W","718")=="CONFIG_REQUIRED"&&link.Writes.Count==writes,"Cached readiness bypassed BeforeMove");
            confirmed=true;
            PreflightAssert(session.MoveRecoverable("W","718")=="TARGET"&&session.LastPingTag=="00000002","Configuration rejection retained old preparation");
        });
        cases.Add("Suspend drains before manual CFG",()=>{
            foreach(int delay in new[]{1200,-1}) {
                var link=new FakeCarLink();var session=PreflightSession(link,delay,true);
                IdlePreparation(session,link,500);IdlePreparation(session,link,400);
                long requested=link.Now;session.SuspendPreparation();
                PreflightAssert(link.Now-requested==(delay<0?1400:800),"Suspend did not wait only the remaining preflight lifetime");
                PreflightAssert(link.Read()==""&&PingCount(link)==1,"Suspend left a due PONG unread or retransmitted PING");
                // Mirror console ordering without coupling standalone sender tests to CarConfigClient.
                PreflightAssert(session.Quiet(500,4000),"CFG quiet failed");
                link.Write(ResultFrame("@CFG,I,ABCDEF12"));
                PreflightAssert(link.WriteTimes[1]>=(delay<0?2300:2200),"CFG overlapped outstanding preflight");
                link.OnWrite=(s,f)=>{if(s.StartsWith("@PING,"))f.Later(20,s.Replace("@PING,","@PONG,"));
                    else if(s.StartsWith("@MOVE,"))f.Later(10,ResultFrame(ResultBody.Replace("1234ABCD",session.LastPingTag)));};
                PreflightAssert(session.MoveRecoverable("W","718")=="TARGET"&&session.LastPingTag=="00000002","Suspend retained a usable old tag");
            }
        });
        cases.Add("manual Ping drains and stores a fresh tag",()=>{
            var link=new FakeCarLink();var session=PreflightSession(link,1200,true);
            IdlePreparation(session,link,500);IdlePreparation(session,link,400);
            PreflightAssert(session.Ping(1),"Explicit PING failed");
            PreflightAssert(PingCount(link)==2&&link.WriteTimes[1]==2200&&session.LastPingTag=="00000002","Explicit PING overlapped background reply or retained its nonce");
            IdlePreparation(session,link,250);long requested=link.Now;
            PreflightAssert(session.MoveRecoverable("W","718")=="TARGET"&&PingCount(link)==2&&LastMoveTime(link)==requested,"Explicit PING confirmation was not reused");
        });
        cases.Add("lost idle PING bounded / foreground five-attempt fallback",()=>{
            var link=new FakeCarLink();var session=PreflightSession(link,-1,false);
            IdlePreparation(session,link,500);
            for(int i=0;i<2000;i++)IdlePreparation(session,link,10);
            PreflightAssert(PingCount(link)==1,"Lost background PING caused an idle retry flood");
            long requested=link.Now;
            PreflightAssert(session.MoveRecoverable("W","718")=="NO_LINK","Lost handshake allowed MOVE");
            PreflightAssert(PingCount(link)==6&&session.LastPingAttempts==5&&MoveCount(link)==0&&link.Now-requested==9000,"Foreground fallback did not use five bounded attempts");
        });
        cases.Add("pending timeout fallback keeps original deadline",()=>{
            var link=new FakeCarLink();var session=PreflightSession(link,-1,true);
            IdlePreparation(session,link,500);IdlePreparation(session,link,1000);
            link.OnWrite=(s,f)=>{if(s.StartsWith("@PING,"))f.Later(20,s.Replace("@PING,","@PONG,"));
                else if(s.StartsWith("@MOVE,"))f.Later(10,ResultFrame(ResultBody.Replace("1234ABCD",session.LastPingTag)));};
            PreflightAssert(session.MoveRecoverable("W","718")=="TARGET","Pending timeout fallback failed");
            PreflightAssert(PingCount(link)==2&&link.WriteTimes[1]==2300&&LastMoveTime(link)==2570,"Pending timeout restarted the 1800 ms deadline");
        });
        cases.Add("Suspend permits a new idle attempt after timeout",()=>{
            var link=new FakeCarLink();var session=PreflightSession(link,-1,false);
            IdlePreparation(session,link,500);IdlePreparation(session,link,1800);IdlePreparation(session,link,20000);
            PreflightAssert(PingCount(link)==1,"Timed-out preparation retried itself");
            long before=link.Now;session.SuspendPreparation();IdlePreparation(session,link,0);
            PreflightAssert(link.Now==before&&PingCount(link)==2,"Explicit suspension did not reset failed preparation");
        });
        cases.Add("prepared MOVE lost RESULT recovers without MOVE replay",()=>{
            foreach(bool recover in new[]{false,true}) {
                var link=new FakeCarLink();var session=PreflightSession(link,20,false);
                ReadyPreparation(session,link);long requested=link.Now;
                link.OnWrite=(s,f)=>{
                    if(s.StartsWith("@MOVE,"))f.Later(20,"@ACK,W,718,CNT\r\n");
                    else {PreflightAssert(s==ResultFrame("@RESULT,00000001"),"Recovery sent something other than a read-only query for the prepared tag");
                        if(recover)f.Later(20,ResultFrame(ResultBody.Replace("1234ABCD","00000001")));}
                };
                PreflightAssert(session.MoveRecoverable("W","718")== (recover?"TARGET":"UNCERTAIN"),"Prepared recovery outcome mismatch");
                PreflightAssert(LastMoveTime(link)==requested&&MoveCount(link)==1&&PingCount(link)==1&&QueryTimes(link).Count==(recover?1:5),"Recovery repeated handshake/MOVE or lost query budget");
                if(!recover) {
                    PreflightAssert(link.Now-requested==14000&&session.Uncertain,"Recovery escaped original deadline/lock");
                    int writes=link.Writes.Count;IdlePreparation(session,link,20000);
                    PreflightAssert(session.MoveRecoverable("W","718")=="LOCKED"&&link.Writes.Count==writes,"Uncertain motion permitted idle preparation or replay");
                }
            }
        });
        cases.Add("emergency invalidates prepared tag",()=>{
            var link=new FakeCarLink();var session=PreflightSession(link,20,true);
            ReadyPreparation(session,link);session.EmergencyRequested=()=>true;session.Poll();
            int writes=link.Writes.Count;IdlePreparation(session,link,20000);
            PreflightAssert(session.Uncertain&&session.MoveRecoverable("W","718")=="LOCKED"&&link.Writes.Count==writes,"Emergency allowed preflight or MOVE");
            session.EmergencyRequested=()=>false;session.Confirm();
            PreflightAssert(session.MoveRecoverable("W","718")=="TARGET"&&PingCount(link)==2&&session.LastPingTag=="00000002","Emergency retained the prepared tag after CONFIRM");
        });
        cases.Add("receive uncertainty invalidates prepared tag before CONFIRM",()=>{
            var link=new FakeCarLink();var session=PreflightSession(link,20,true);
            ReadyPreparation(session,link);link.Later(0,new string('x',4097));session.Poll();
            PreflightAssert(session.Uncertain,"Oversized RX did not latch uncertainty");
            // No idle call intervenes: invalidation belongs to the state transition itself.
            session.Confirm();
            PreflightAssert(session.MoveRecoverable("W","718")=="TARGET"&&PingCount(link)==2&&session.LastPingTag=="00000002","Uncertain RX retained old prepared tag across CONFIRM");
        });
        var failures=new List<string>();
        foreach(var test in cases) {
            try{test.Value();Console.WriteLine("PASS: idle preflight: "+test.Key);}
            catch(Exception ex){string failure=test.Key+": "+ex.Message;failures.Add(failure);Console.WriteLine("FAIL: idle preflight: "+failure);}
        }
        if(failures.Count>0)throw new Exception("Idle preflight failures ("+failures.Count+"): "+string.Join("; ",failures));
    }
    public static void Run() {
        RecoverableResults();
        RecoverySafety();
        DiagnosticFragmentReplay();
        Assert(CarSession.DoneReason(Done,"Q","1200")=="TARGET");
        Assert(CarSession.DoneReason(Done,"C","1200")==null);
        Assert(CarSession.DoneReason(Done.Substring(0,Done.Length-1),"Q","1200")==null);
        Assert(CarSession.DoneReason(Done+"junk","Q","1200")==null);
        string mmDone=Done.Replace("REQ=1200,UNIT=CNT","REQ=100,UNIT=MM,TARGET_CNT=1018");
        Assert(CarSession.DoneReason(mmDone,"Q","100","MM")=="TARGET");
        Assert(CarSession.DoneReason(mmDone,"Q","100","CNT")==null);
        var mmLink=new FakeCarLink();int mmTag=0;
        mmLink.OnWrite=(s,f)=>{
            if(s.StartsWith("@PING,"))f.Later(20,s.Replace("@PING,","@PONG,"));
            else {Assert(s=="@MOVE,Q,100,MM\r\n");f.Later(20,"@ACK,Q,100,MM\r\n");f.Later(100,mmDone+"\r\n");}
        };
        var mmSession=new CarSession(mmLink,s=>{},s=>{},()=> (++mmTag).ToString("X8"));
        Assert(mmSession.Move("Q","100","MM")=="TARGET" && !mmSession.Uncertain);
        // Replay the field failure: telemetry and a full DONE separated by CR only.
        string fieldDone="@DONE,C,TARGET,REQ=100,UNIT=MM,TARGET_CNT=1019,BRAKE=989.50,ENC=1077.50,DX=-539.75,DY=-537.75,DR=1.75,DS=-1.75,Q1=-1081,Q2=-2,Q3=-1074,Q4=-2";
        foreach(string ending in new[]{"\r","\n","\r\n","split","missing","truncated"}) {
            var fieldLink=new FakeCarLink();
            var fieldSession=new CarSession(fieldLink,s=>{},s=>{},()=>"ABCDEF12");
            fieldSession.Padding=32;
            fieldLink.OnWrite=(s,f)=>{
                Assert(s.StartsWith(new string(' ',32)));
                s=s.Trim();
                if(s.StartsWith("@PING,")){f.Later(20,s.Replace("@PING,","@PONG,")+"\r\n");return;}
                Assert(s=="@MOVE,C,100,MM");
                f.Later(20,"@ACK,C,100,MM\r\n");
                if(ending=="split"){
                    f.Later(100,"FREE_RAM_START=840\r");
                    f.Later(120,"\n"+fieldDone.Substring(0,60));
                    f.Later(150,fieldDone.Substring(60)+"\r");f.Later(170,"\n");
                } else if(ending=="missing")f.Later(100,fieldDone);
                else if(ending=="truncated")f.Later(100,fieldDone.Substring(0,90)+"\r\n");
                else f.Later(100,"FREE_RAM_START=840"+ending+fieldDone+ending);
            };
            bool valid=ending!="missing" && ending!="truncated";
            Assert((fieldSession.Move("C","100","MM")=="TARGET")==valid);
            Assert(fieldSession.Uncertain==!valid);
            Assert(fieldLink.Writes.FindAll(s=>s.Contains("@MOVE,")).Count==1);
        }
        byte[] rawBytes={0,10,13,65,128,255};
        Assert(Convert.ToBase64String(Encoding.GetEncoding(28591).GetBytes(Encoding.GetEncoding(28591).GetString(rawBytes)))==Convert.ToBase64String(rawBytes));
        foreach(string scenario in new[]{"normal","fragmented","noack","lostdone","mismatch","timeout","badcmd","partialdone","wrongtag","busy","reboot","rebootglued","writeerror"}) {
            var fake=new FakeCarLink();int seq=0;
            var session=new CarSession(fake,s=>{},s=>{},()=> (++seq).ToString("X8"));
            fake.OnWrite=(s,f)=>{
                s=s.Trim();
                if(s.StartsWith("@PING,")) {
                    if(scenario=="busy")f.Later(20,"@ERR,BUSY\r\n");
                    else if(scenario=="wrongtag")f.Later(20,"@PONG,FFFFFFFF\r\n");
                    else f.Later(20,s.Replace("@PING,","@PONG,")+"\r\n");
                } else {
                    if(scenario=="writeerror")throw new IOException("partial write");
                    if(scenario=="badcmd"){f.Later(20,"@ERR,BAD_CMD\r\n");return;}
                    if(scenario=="mismatch"){f.Later(20,"@ACK,E,1200,CNT\r\n");return;}
                    if(scenario!="noack")f.Later(20,"@ACK,Q,1200,CNT\r\n");
                    if(scenario=="lostdone")return;
                    if(scenario=="reboot"){f.Later(100,"MECANUM UNIVERSAL V6.3 COMM READY\r\n");return;}
                    if(scenario=="rebootglued"){f.Later(100,"MECANUM UNIVERSAL V6.3 COMM READY"+Done+"\r\n");return;}
                    if(scenario=="fragmented"){f.Later(100,Done.Substring(0,40));f.Later(180,Done.Substring(40)+"\r\n");}
                    else if(scenario=="partialdone")f.Later(100,Done.Substring(0,60)+"\r\n");
                    else f.Later(100,(scenario=="timeout"?Done.Replace(",TARGET,",",TIMEOUT,"):Done)+"\r\n");
                }
            };
            string outcome;
            try {outcome=session.Move("Q","1200");}catch(IOException){outcome="writeerror";}
            int moves=fake.Writes.FindAll(s=>s.Contains("@MOVE,")).Count;
            Assert(moves==(scenario=="wrongtag"||scenario=="busy"?0:1));
            bool success=scenario=="normal"||scenario=="fragmented"||scenario=="noack";
            Assert((outcome=="TARGET")==success);
            if(moves==1&&!success){Assert(session.Uncertain);Assert(session.Move("Q","1200")=="LOCKED");Assert(fake.Writes.FindAll(s=>s.Contains("@MOVE,")).Count==1);}
        }
        var quietLink=new FakeCarLink();int tag=0;
        for(int i=100;i<=1000;i+=100)quietLink.Later(i,"telemetry\r\n");
        var quietSession=new CarSession(quietLink,s=>{},s=>{},()=> (++tag).ToString("X8"));
        quietSession.Ping(1);Assert(quietLink.WriteTimes[0]>=1500);
        var idleLink=new FakeCarLink();
        idleLink.OnWrite=(s,f)=>f.Later(20,s.Trim().Replace("@PING,","@PONG,")+"\r\n");
        var idleSession=new CarSession(idleLink,s=>{},s=>{},()=>"12345678");
        idleLink.Sleep(5000);long idleStart=idleLink.Now;
        Assert(idleSession.Ping(1));
        Assert(idleLink.WriteTimes[0]==idleStart); // Already idle: no fresh 500 ms wait.
        long pongAt=idleLink.Now;
        Assert(idleSession.Quiet(250,4000));
        Assert(idleLink.Now-pongAt>=250); // A new reply still requires its real quiet interval.
        var checkLink=new FakeCarLink();tag=0;
        checkLink.OnWrite=(s,f)=>f.Later(20,s.Trim().Replace("@PING,","@PONG,")+"\r\n");
        var checkSession=new CarSession(checkLink,s=>{},s=>{},()=> (++tag).ToString("X8"));
        checkSession.Check(20);Assert(checkLink.Writes.Count==20 && checkLink.Writes.TrueForAll(s=>s.StartsWith("@PING,")));
        Console.WriteLine("PASS: transactions, fragmentation, stale PONG, busy, partial write, no MOVE retries, RX silence, CHECK without motion.");
        IdlePreflight();
    }
}
