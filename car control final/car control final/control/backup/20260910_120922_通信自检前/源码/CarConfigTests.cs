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
        Check(config.Values.SequenceEqual(CarParameterSchema.Defaults));
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
            Check(failed==(failure!="none"));
            Check(client.IsVerified(target)==!failed);
            Check(commits==(failure=="set_timeout"?0:1));
            Check(client.Read().SequenceEqual(failure=="set_timeout"?CarParameterSchema.Defaults:target));
            Check(link.Writes.Count(w=>w.Contains("@MOVE,"))==2);
            link.Later(0,"MECANUM UNIVERSAL V6.3 COMM READY\r\n");session.Poll();
            Check(!client.IsVerified(target));
            Check(session.Move("D","700")=="CONFIG_REQUIRED");
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
        Console.WriteLine("PASS: cached configuration, two moves without CFG queries, restart invalidation, CRC/readback/apply, lost commit ACK, host MM conversion; no hardware opened.");
    }
}
