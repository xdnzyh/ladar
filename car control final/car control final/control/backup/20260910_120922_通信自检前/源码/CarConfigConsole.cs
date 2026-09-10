using System;
using System.IO;
using System.IO.Ports;
using System.Text;
using System.Text.RegularExpressions;
using System.Globalization;
using System.Threading;

class CarConfigConsole {
    static int Main(string[] args) {
        if(args.Length==1&&args[0]=="--test"){
            try{CarConfigTests.Run();CarLinkTests.Run();return 0;}
            catch(Exception ex){Console.WriteLine(ex);return 1;}
        }
        Console.OutputEncoding=Encoding.UTF8;
        string dir=AppDomain.CurrentDomain.BaseDirectory;
        string path=Path.Combine(dir,"底盘参数.json");
        string logs=Path.Combine(dir,"logs");Directory.CreateDirectory(logs);
        string stamp=DateTime.Now.ToString("yyyyMMdd_HHmmss_fff");
        using(var events=new StreamWriter(Path.Combine(logs,stamp+"_config.txt"),false,new UTF8Encoding(false)))
        using(var raw=new FileStream(Path.Combine(logs,stamp+"_config_rx.bin"),FileMode.CreateNew,FileAccess.Write)) {
            Action<string> note=s=>{Console.WriteLine(s);events.WriteLine(DateTime.Now.ToString("O")+" "+s);events.Flush();};
            try {
                CarParameterFile config=CarParameterFile.Load(path);
                note("底盘免烧录参数工具 CONFIG1。请先退出占用同一串口的发送器/上位机。连接不自动下发，不自动移动。");
                note("配置文件："+path);
                note("可用端口："+string.Join(", ",SerialPort.GetPortNames()));
                Console.Write("端口：");string name=Console.ReadLine();if(name==null)return 0;
                using(var link=new SerialCarLink(name.Trim())) {
                    var session=new CarSession(link,note,s=>{byte[] b=Encoding.GetEncoding(28591).GetBytes(s);raw.Write(b,0,b.Length);raw.Flush();},
                        ()=>Guid.NewGuid().ToString("N").Substring(0,8).ToUpperInvariant());
                    session.Padding=32;
                    var client=new CarConfigClient(link,session,note);
                    session.BeforeMove=()=>client.IsVerified(config.Values);
                    session.Quiet(1500,5000);
                    note("READ读取核对 / LOAD重载JSON / APPLY提交并回读 / EXPORT导出实际配置 / PING / D 100 MM / D 700 CNT / CONFIRM / EXIT");
                    note("参数缓存版 RX_IDLE=1：连接后READ/APPLY成功一次，后续运动不再查询参数；已满足的静默时间不重复等待。");
                    note("MM由电脑换算为CNT。LOAD若改变执行参数则需APPLY；只改距离系数无需写设备。动作等待/执行中Esc请求停止。");
                    var input=new StringBuilder();Console.Write("> ");
                    while(true) {
                        session.Poll();
                        if(!Console.KeyAvailable){Thread.Sleep(20);continue;}
                        var key=Console.ReadKey(true);
                        if(key.Key==ConsoleKey.Backspace){if(input.Length>0){input.Length--;Console.Write("\b \b");}continue;}
                        if(key.Key!=ConsoleKey.Enter){if(!char.IsControl(key.KeyChar)){input.Append(key.KeyChar);Console.Write(key.KeyChar);}continue;}
                        Console.WriteLine();string command=input.ToString().Trim().ToUpperInvariant();input.Length=0;
                        if(command=="EXIT")break;
                        try {
                            if(command=="LOAD"){
                                config=CarParameterFile.Load(path);
                                if(!client.IsVerified(config.Values))client.Invalidate();
                                note(client.IsVerified(config.Values)?"已重载，设备执行参数未变，沿用已确认状态。":"已重载，执行参数待确认，请READ/APPLY。");
                            }
                            else if(command=="CONFIRM")session.Confirm();
                            else if(command=="PING")note(session.Ping(5)?"握手成功。":"握手未完成。");
                            else if(command=="READ"||command=="EXPORT") {
                                var actual=client.Read();int differences=0;
                                for(int i=0;i<actual.Length;i++)if(actual[i]!=config.Values[i]){
                                    differences++;note(CarParameterSchema.Names[i]+": 设备="+((decimal)actual[i]/CarParameterSchema.Scale[i])+"，文件="+((decimal)config.Values[i]/CarParameterSchema.Scale[i]));
                                }
                                note("回读完整，CRC="+CarConfigClient.Hash(actual)+"，与文件不同项="+differences);
                                if(command=="EXPORT"){
                                    string save=Path.Combine(dir,"底盘参数_回读_"+DateTime.Now.ToString("yyyyMMdd_HHmmss_fff")+".json");
                                    config.Save(save,actual);note("已导出设备执行参数；距离系数沿用当前电脑文件："+save);
                                }
                            }
                            else if(command=="APPLY"){
                                if(session.Uncertain)throw new Exception("上一运动未确认，先现场确认停稳再CONFIRM。");
                                client.Apply(config.Values);
                            }
                            else {
                                var m=Regex.Match(command,@"^([WSADQEZC])\s+([0-9]+)\s+(MM|CNT)$");
                                int value;
                                if(!m.Success||!int.TryParse(m.Groups[2].Value,out value)||value<=0)throw new Exception("格式：D 100 MM 或 D 700 CNT；其他命令见帮助。");
                                if(session.Uncertain)throw new Exception("上一运动未确认，先现场确认再CONFIRM。");
                                if(!client.IsVerified(config.Values))throw new Exception("参数尚未确认，请先READ/APPLY成功一次；后续动作沿用确认状态。");
                                string mode=m.Groups[1].Value;
                                long counts=m.Groups[3].Value=="MM"?config.ToCounts(mode,value):value;
                                note("请求="+command+"，CNT/mm="+config.CountsPerMm[mode]+", 发送CNT="+counts+", CONFIG_CRC="+CarConfigClient.Hash(config.Values));
                                // Each request is single-shot. The configurable firmware timeout may exceed the legacy wait.
                                session.EmergencyRequested=()=>{
                                    while(Console.KeyAvailable)if(Console.ReadKey(true).Key==ConsoleKey.Escape)return true;
                                    return false;
                                };
                                try {session.Move(mode,counts.ToString(CultureInfo.InvariantCulture),"CNT",Math.Max(14000,config.Values[76]+6000));}
                                finally {session.EmergencyRequested=null;}
                            }
                        }catch(Exception ex){note("未确认成功："+ex.Message);}
                        Console.Write("> ");
                    }
                }
            }catch(Exception ex){note("工具退出："+ex.Message);Console.WriteLine("回车关闭。");Console.ReadLine();return 1;}
        }
        return 0;
    }
}
