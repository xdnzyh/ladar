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
    public static void Run() {
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
        foreach(string scenario in new[]{"normal","fragmented","noack","lostdone","mismatch","timeout","badcmd","partialdone","wrongtag","busy","reboot","writeerror"}) {
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
    }
}
