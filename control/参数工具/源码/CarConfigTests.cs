using System;
using System.IO;
using System.Linq;
using System.Text;

class CarConfigTests {
    static void Check(bool ok){if(!ok)throw new Exception("Config test assertion failed");}
    public static void Run() {
        Check(CarConfigClient.Crc(Encoding.ASCII.GetBytes("123456789"))==0x29B1);
        Check(CarConfigClient.Hash(CarParameterSchema.Defaults)==5809);
        Check(CarConfigClient.Parse("@CFG,I,12345678,1,95,0,0000")==null);
        var config=CarParameterFile.Load(Path.Combine(AppDomain.CurrentDomain.BaseDirectory,"底盘参数.json"));
        // Validate the user's file, but use fixed fixtures without changing it.
        CarParameterFile.Validate(config.Values);
        config.Values=(int[])CarParameterSchema.Defaults.Clone();
        config.CountsPerMm["D"]=7.3864m;
        Check(config.ToCounts("D",100)==739);
        config.CountsPerMm["D"]=8m;Check(config.ToCounts("D",100)==800);
        var invalid=(int[])config.Values.Clone();invalid[9]=120;
        bool rejected=false;try{CarParameterFile.Validate(invalid);}catch{rejected=true;}Check(rejected);
        foreach(string failure in new[]{"none","set_timeout","commit_reply_lost"}) {
            var link=new FakeCarLink();
            var active=(int[])CarParameterSchema.Defaults.Clone();
            int[] staged=null;bool corrupt=false;int commits=0;
            link.OnWrite=(wire,f)=>{
                Check(wire.StartsWith(new string(' ',32)));
                string body=wire.Trim();
                if(body.StartsWith("@PING,")){f.Later(25,body.Replace("@PING,","@PONG,")+"\r\n");return;}
                if(body.StartsWith("@MOVE,")){
                    f.Later(25,"@ACK,D,700,CNT\r\n@DONE,D,TARGET,REQ=700,UNIT=CNT,BRAKE=625.00,ENC=700.00,DX=0.00,DY=-700.00,DR=0.00,DS=0.00,Q1=-700,Q2=700,Q3=-700,Q4=700\r\n");return;
                }
                var q=CarConfigClient.Parse(body);Check(q!=null);
                string op=q[1],tag=q[2],reply="@CFG,"+op+","+tag;
                switch(op) {
                    case "I": reply+=",1,95,"+CarConfigClient.Hash(active);break;
                    case "G":
                        int start=int.Parse(q[3]);reply+=","+start;
                        for(int i=start;i<Math.Min(start+8,95);i++)reply+=","+active[i];break;
                    case "T":
                        reply+=","+q[3]+","+new string(Enumerable.Range(0,int.Parse(q[3])).Select(i=>(char)('0'+i%10)).ToArray());break;
                    case "B":staged=(int[])active.Clone();reply+=","+CarConfigClient.Hash(active);break;
                    case "S":
                        staged[int.Parse(q[3])]=int.Parse(q[4]);
                        reply+=","+q[3]+","+q[4];corrupt=failure=="set_timeout";break;
                    case "C":
                        Check(CarConfigClient.Hash(staged).ToString()==q[3]);
                        active=(int[])staged.Clone();commits++;reply+=","+CarConfigClient.Hash(active);
                        if(failure=="commit_reply_lost")return;break;
                    case "A":reply="@CFG,A,"+tag+","+CarConfigClient.Hash(active);staged=null;break;
                    default:throw new Exception("Unexpected request");
                }
                f.Later(25,corrupt?"@CFG,S,"+tag+",0,0,0000\r\n":CarConfigClient.Frame(reply));corrupt=false;
            };
            var session=new CarSession(link,s=>{},s=>{},()=>"ABCDEF12");
            session.Padding=32;
            var client=new CarConfigClient(link,session,s=>{});
            session.BeforeMove=()=>client.IsVerified(CarParameterSchema.Defaults);
            Check(!client.IsVerified(CarParameterSchema.Defaults));
            Check(session.Move("D","700")=="CONFIG_REQUIRED" && link.Writes.Count==0);
            Check(client.Read().SequenceEqual(CarParameterSchema.Defaults));
            Check(client.IsVerified(CarParameterSchema.Defaults));
            Check(client.LinkTest()==6);
            Check(client.IsVerified(CarParameterSchema.Defaults));
            Check(link.Writes.Count(w=>w.Contains("@CFG,T,"))==6);
            int configWrites=link.Writes.Count(w=>w.Contains("@CFG,"));
            for(int move=0;move<2;move++) {
                long started=link.Now;
                Check(session.Move("D","700")=="TARGET");
                Check(link.Now-started<1000);
            }
            Check(link.Writes.Count(w=>w.Contains("@CFG,"))==configWrites);
            var normalWrite=link.OnWrite;
            link.OnWrite=(wire,f)=>{
                if(wire.Contains("@PING,"))f.Later(10,"MECANUM UNIVERSAL V6.3 COMM READY\r\n");
                normalWrite(wire,f);
            };
            Check(session.Move("D","700")=="CONFIG_REQUIRED");
            Check(link.Writes.Count(w=>w.Contains("@MOVE,"))==2);
            link.OnWrite=normalWrite;
            Check(client.Read().SequenceEqual(CarParameterSchema.Defaults));
            var target=(int[])active.Clone();target[76]=9000;
            bool failed=false;try{client.Apply(target);}catch{failed=true;}
            Check(failed==(failure=="set_timeout"));
            Check(client.IsVerified(target)==!failed);
            Check(commits==(failure=="set_timeout"?0:1));
            Check(client.Read().SequenceEqual(failure=="set_timeout"?CarParameterSchema.Defaults:target));
            Check(link.Writes.Count(w=>w.Contains("@MOVE,"))==2);
            link.Later(0,"MECANUM UNIVERSAL V6.3 COMM READY\r\n");session.Poll();
            Check(!client.IsVerified(target));
            Check(session.Move("D","700")=="CONFIG_REQUIRED");
        }
        foreach(string failure in new[]{"tail_lost","bad_crc","wrong_payload","wrong_nonce","old_firmware"}) {
            var link=new FakeCarLink();
            link.OnWrite=(wire,f)=>{
                var q=CarConfigClient.Parse(wire.Trim());Check(q!=null && q[1]=="T");
                int length=int.Parse(q[3]);
                string reply="@CFG,T,"+q[2]+","+length+","+new string(Enumerable.Range(0,length).Select(i=>(char)('0'+i%10)).ToArray());
                if(failure=="old_firmware") {f.Later(20,CarConfigClient.Frame("@CFG,E,"+q[2]+",4"));return;}
                if(length==128) {
                    if(failure=="wrong_payload")reply=reply.Substring(0,reply.Length-1)+"X";
                    if(failure=="wrong_nonce")reply=reply.Replace(q[2],"00000000");
                    string frame=CarConfigClient.Frame(reply);
                    if(failure=="tail_lost")frame=frame.Substring(0,frame.Length-12);
                    if(failure=="bad_crc")frame=frame.Substring(0,frame.Length-6)+"ZZZZ\r\n";
                    f.Later(20,frame);return;
                }
                f.Later(20,CarConfigClient.Frame(reply));
            };
            var session=new CarSession(link,s=>{},s=>{},()=>"1234ABCD");
            var client=new CarConfigClient(link,session,s=>{});
            if(failure=="old_firmware") {
                bool stopped=false;try{client.LinkTest();}catch(NotSupportedException){stopped=true;}
                Check(stopped && link.Writes.Count==1);
            } else Check(client.LinkTest()==4 && link.Writes.Count==6);
            Check(!client.IsVerified(CarParameterSchema.Defaults));
            Check(link.Writes.All(w=>w.Contains("@CFG,T,")));
        }
        foreach(int escapeAt in new[]{100,900}) {
            var link=new FakeCarLink();
            link.OnWrite=(s,f)=>{
                if(s.StartsWith("@PING,"))f.Later(20,s.Replace("@PING,","@PONG,"));
                else if(s.StartsWith("@MOVE,"))f.Later(20,"@ACK,D,700,CNT\r\n");
                else if(s=="!\r\nX\r\n")f.Later(20,"@DONE,D,EMERGENCY,REQ=700,UNIT=CNT,BRAKE=10.00,ENC=10.00,DX=0.00,DY=-10.00,DR=0.00,DS=0.00,Q1=-10,Q2=10,Q3=-10,Q4=10\r\n");
            };
            var session=new CarSession(link,s=>{},s=>{},()=>"1234ABCD");
            session.EmergencyRequested=()=>link.Now>=escapeAt;
            string result=session.Move("D","700");
            Check(session.Uncertain && link.Writes.Count(s=>s=="!\r\nX\r\n")==1);
            Check(link.Writes.Count(s=>s.StartsWith("@MOVE,"))==(escapeAt==100?0:1));
            Check(result==(escapeAt==100?"NO_LINK":"EMERGENCY"));
        }
        Console.WriteLine("PASS: LINKTEST length/content/CRC/nonce/truncation/unsupported checks, cached configuration, two moves without CFG queries, restart invalidation, CRC/readback/apply, lost commit ACK, host MM conversion; no hardware opened.");
        Recovery();
    }
    static void Recovery() {
        foreach(string fault in new[]{"page_lost","page_crc","page_partial","page_dead","begin_lost","set_lost","set_request_lost","commit_lost","commit_request_lost","reboot","rejected"}) {
            var link=new FakeCarLink();var active=(int[])CarParameterSchema.Defaults.Clone();
            int[] staged=null;string owner=null;bool injected=false;int commits=0,commitRequests=0,begins=0,sets=0,pages=0;
            var readTags=new System.Collections.Generic.HashSet<string>();
            link.OnWrite=(wire,f)=>{
                var q=CarConfigClient.Parse(wire.Trim());Check(q!=null);
                string op=q[1],tag=q[2],reply="@CFG,"+op+","+tag;
                if(op=="I"||op=="G")Check(readTags.Add(tag));
                if(fault=="reboot"&&op=="G"&&!injected){injected=true;f.Later(20,"MECANUM UNIVERSAL V6.3 COMM READY\r\n");return;}
                if(fault=="rejected"&&op=="B"){begins++;f.Later(20,CarConfigClient.Frame("@CFG,E,"+tag+",2"));return;}
                switch(op) {
                    case "I":reply+=",1,95,"+CarConfigClient.Hash(active);break;
                    case "G":
                        int start=int.Parse(q[3]);reply+=","+start;
                        for(int i=start;i<Math.Min(95,start+8);i++)reply+=","+active[i];
                        if(start==0){pages++;if(fault=="page_dead")return;
                            if(!injected&&fault.StartsWith("page_")){injected=true;
                                if(fault=="page_lost")return;
                                string damaged=CarConfigClient.Frame(reply);
                                f.Later(20,fault=="page_crc"?damaged.Substring(0,damaged.Length-6)+"0000\r\n":damaged.Substring(0,25));return;
                            }}break;
                    case "B":
                        begins++;if(staged==null){owner=tag;staged=(int[])active.Clone();}Check(owner==tag);
                        reply+=","+CarConfigClient.Hash(active);
                        if(fault=="begin_lost"&&!injected){injected=true;return;}break;
                    case "S":
                        sets++;Check(owner==tag);
                        if(fault=="set_request_lost"&&!injected){injected=true;return;}
                        staged[int.Parse(q[3])]=int.Parse(q[4]);reply+=","+q[3]+","+q[4];
                        if(fault=="set_lost"&&!injected){injected=true;return;}break;
                    case "C":
                        commitRequests++;Check(owner==tag);
                        if(fault=="commit_request_lost")return;
                        Check(CarConfigClient.Hash(staged).ToString()==q[3]);active=(int[])staged.Clone();staged=null;commits++;
                        reply+=","+CarConfigClient.Hash(active);if(fault=="commit_lost")return;break;
                    case "A":staged=null;reply+=","+CarConfigClient.Hash(active);break;
                    default:throw new Exception("Unexpected configuration test command");
                }
                f.Later(20,CarConfigClient.Frame(reply));
            };
            var session=new CarSession(link,s=>{},s=>{},()=>"ABCDEF12");
            var client=new CarConfigClient(link,session,s=>{});
            var wanted=(int[])active.Clone();wanted[19]=600;
            bool failed=false;try{client.Apply(wanted);}catch{failed=true;}
            bool expectedFailure=fault=="page_dead"||fault=="reboot"||fault=="rejected"||fault=="commit_request_lost";
            Check(failed==expectedFailure);Check(client.IsVerified(wanted)==!failed);
            Check(commits==(failed?0:1));Check(commitRequests<2);
            Check(link.Writes.All(w=>!w.Contains("@MOVE,")));
            if(fault=="page_dead")Check(pages==3&&begins==0);
            if(fault=="begin_lost")Check(begins==2);
            if(fault=="set_lost"||fault=="set_request_lost")Check(sets==2);
            if(fault=="rejected")Check(begins==1);
            Console.WriteLine("PASS: CONFIG recovery "+fault);
        }
    }
}
