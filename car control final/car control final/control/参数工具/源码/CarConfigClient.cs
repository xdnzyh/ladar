using System;
using System.IO;
using System.Text;
using System.Linq;
using System.Globalization;
using System.Collections.Generic;
using System.Web.Script.Serialization;

class CarParameterFile {
    public int[] Values;
    public Dictionary<string,decimal> CountsPerMm;
    static decimal Numeric(object value) {
        if(value==null||value is bool||value is string)throw new Exception("参数必须是JSON数值，不能是布尔、空值或字符串。");
        return Convert.ToDecimal(value,CultureInfo.InvariantCulture);
    }
    public static CarParameterFile Load(string path) {
        var root=new JavaScriptSerializer().Deserialize<Dictionary<string,object>>(File.ReadAllText(path,Encoding.UTF8));
        if(root.Count!=3 || !(root["schema"] is int) || (int)root["schema"]!=1)throw new Exception("配置schema必须为1，且仅包含schema/parameters/counts_per_mm。");
        var values=(Dictionary<string,object>)root["parameters"];
        var mm=(Dictionary<string,object>)root["counts_per_mm"];
        if(values.Count!=CarParameterSchema.Names.Length || mm.Count!=8)throw new Exception("参数字段不完整或含未知字段。");
        var result=new CarParameterFile {Values=new int[CarParameterSchema.Names.Length], CountsPerMm=new Dictionary<string,decimal>()};
        for(int i=0;i<result.Values.Length;i++) {
            decimal v=Numeric(values[CarParameterSchema.Names[i]])*CarParameterSchema.Scale[i];
            if(v!=decimal.Truncate(v) || v<CarParameterSchema.Minimum[i] || v>CarParameterSchema.Maximum[i])
                throw new Exception("参数精度或范围不合法："+CarParameterSchema.Names[i]);
            result.Values[i]=(int)v;
        }
        foreach(char mode in "WSADQEZC") {
            decimal v=Numeric(mm[mode.ToString()]);
            if(v<0.0001m || v>1000m || v*10000!=decimal.Truncate(v*10000))throw new Exception("CNT/mm须为0.0001～1000，最多4位小数。");
            result.CountsPerMm[mode.ToString()]=v;
        }
        Validate(result.Values);return result;
    }
    public static void Validate(int[] v) {
        if(v.Length!=95)throw new Exception("配置数量错误。");
        for(int i=0;i<v.Length;i++)if(v[i]<CarParameterSchema.Minimum[i]||v[i]>CarParameterSchema.Maximum[i])throw new Exception("参数范围错误。");
        for(int p=0;p<4;p++) {
            int b=p*11;
            if(v[b]<v[b+1]||v[b+1]<v[b+2]||v[b+2]<v[b+6]||v[b+9]>v[b+10])throw new Exception("速度档位顺序或PWM范围错误。");
            foreach(int k in new[]{3,4,5,7})if(v[b+k]<v[b+9]||v[b+k]>v[b+10])throw new Exception("前馈PWM超出本类PWM上下限。");
        }
        for(int p=0;p<8;p++){int b=44+p*4;if(v[b]>=v[b+1]||2*v[b+2]>=v[b]||2*v[b+3]>=v[b+1])throw new Exception("刹车区间或提前量不合法。");}
        if(v[80]>=v[79]||v[88]>v[89])throw new Exception("降速区间或航向修正上限不合法。");
    }
    public long ToCounts(string mode,int mm) {
        decimal v=decimal.Round(mm*CountsPerMm[mode],0,MidpointRounding.AwayFromZero);
        if(v<=0 || v>int.MaxValue)throw new Exception("换算CNT越界。");
        return (long)v;
    }
    public void Save(string path,int[] actual) {
        var parameters=new Dictionary<string,decimal>();
        for(int i=0;i<actual.Length;i++)parameters[CarParameterSchema.Names[i]]=(decimal)actual[i]/CarParameterSchema.Scale[i];
        var doc=new Dictionary<string,object>{{"schema",1},{"parameters",parameters},{"counts_per_mm",CountsPerMm}};
        File.WriteAllText(path,new JavaScriptSerializer().Serialize(doc),new UTF8Encoding(false));
    }
}
class CarConfigClient {
    readonly ICarLink link;
    readonly CarSession session;
    readonly Action<string> note;
    int[] verifiedValues;
    int verifiedEpoch=-1;
    public void Invalidate(){verifiedValues=null;verifiedEpoch=-1;}
    public bool IsVerified(int[] wanted) {
        return verifiedValues!=null && verifiedEpoch==session.ConfigurationEpoch && verifiedValues.SequenceEqual(wanted);
    }
    public CarConfigClient(ICarLink l,CarSession s,Action<string> n){link=l;session=s;note=n;}
    public static ushort Crc(byte[] bytes) {
        ushort crc=65535;
        foreach(byte b in bytes) {
            crc^=(ushort)(b<<8);
            for(int i=0;i<8;i++)crc=(ushort)((crc&0x8000)!=0?(crc<<1)^0x1021:crc<<1);
        }
        return crc;
    }
    public static ushort Hash(int[] values) {
        var bytes=new byte[values.Length*2];
        for(int i=0;i<values.Length;i++){bytes[2*i]=(byte)values[i];bytes[2*i+1]=(byte)(values[i]>>8);}
        return Crc(bytes);
    }
    public static string Frame(string body){return body+","+Crc(Encoding.ASCII.GetBytes(body)).ToString("X4")+"\r\n";}
    public static string[] Parse(string line) {
        if(line.Any(c=>c<32||c>126))return null;
        int cut=line.LastIndexOf(',');
        if(cut<0 || line.Length-cut!=5)return null;
        ushort crc;
        if(!ushort.TryParse(line.Substring(cut+1),NumberStyles.AllowHexSpecifier,CultureInfo.InvariantCulture,out crc))return null;
        string body=line.Substring(0,cut);
        if(Crc(Encoding.ASCII.GetBytes(body))!=crc)return null;
        return body.Split(',');
    }
    string[] Exchange(string op,string tag,params string[] args) {
        // I/G only read active values. B/S repeat the same transaction/value;
        // firmware preserves an existing B transaction and assigns S absolutely.
        int attempts=(op=="I"||op=="G"||op=="B"||op=="S")?3:1;
        int epoch=session.ConfigurationEpoch;
        for(int attempt=0;attempt<attempts;attempt++) {
            string requestTag=(op=="I"||op=="G")?Tag():tag;
            try {return ExchangeOnce(op,requestTag,epoch,args);}
            catch(TimeoutException) {
                if(attempt+1==attempts)throw;
                note("配置"+op+"回报未确认，恢复第"+(attempt+1)+"/"+(attempts-1)+"次；不发送运动。");
            }
        }
        throw new TimeoutException("配置恢复未完成，请READ核对。");
    }
    string[] ExchangeOnce(string op,string tag,int epoch,params string[] args) {
        if(!session.Quiet(500,4000))throw new Exception("串口未空闲，配置未发送。");
        if(epoch!=session.ConfigurationEpoch)throw new Exception("配置期间设备重启，请重新READ/APPLY。");
        string body="@CFG,"+op+","+tag+(args.Length==0?"":","+string.Join(",",args));
        if(body.Length+5>47)throw new Exception("配置命令过长。");
        note("TX "+body+" [CRC/PAD32]");
        link.Write(new string(' ',32)+Frame(body));
        long start=link.Now;
        while(link.Now-start<4000) {
            var lines=session.Poll();
            if(epoch!=session.ConfigurationEpoch)throw new Exception("配置期间设备重启，请重新READ/APPLY。");
            foreach(string line in lines) {
                var f=Parse(line);
                if(f==null||f.Length<4||f[0]!="@CFG"||f[2]!=tag)continue;
                if(f[1]=="E") {
                    if(op=="T")throw new NotSupportedException("通信自检被设备拒绝，代码="+f[3]+"；代码2表示设备忙，代码4通常表示仍是未加入LINKTEST的固件。");
                    throw new Exception("设备配置拒绝，代码="+f[3]+"（2忙，3范围，4事务失效，6组合约束，7整组校验）。");
                }
                if(f[1]==op) {
                    if((op=="G"||op=="S")&&(f.Length<4||f[3]!=args[0]))continue;
                    if(op=="S"&&(f.Length<5||f[4]!=args[1]))continue;
                    return f;
                }
            }
            link.Sleep(10);
        }
        throw new TimeoutException(op=="T"?"通信自检应答超时，未收到匹配且CRC正确的完整回报。":"配置应答持续未确认，请READ核对实际值。");
    }
    static string Tag(){return Guid.NewGuid().ToString("N").Substring(0,8).ToUpperInvariant();}
    int Info(string tag) {
        var f=Exchange("I",tag);
        if(f.Length!=6||f[3]!="1"||f[4]!="95")throw new Exception("设备CONFIG版本/参数数量不匹配。");
        return int.Parse(f[5],CultureInfo.InvariantCulture);
    }
    public int ReadHash(){session.SuspendPreparation();return Info(Tag());}
    public int LinkTest() {
        session.SuspendPreparation();
        note("LINKTEST=1 静止通信自检：16/64/128字节有效内容，各2次；不发送运动或修改参数。");
        int total=0;
        foreach(int length in new[]{16,64,128}) {
            int passed=0;
            for(int attempt=1;attempt<=2;attempt++) {
                long start=link.Now;
                try {
                    var f=Exchange("T",Tag(),length.ToString(CultureInfo.InvariantCulture));
                    if(f.Length!=5 || f[3]!=length.ToString(CultureInfo.InvariantCulture) || f[4].Length!=length
                        || f[4].Where((c,i)=>c!=(char)('0'+i%10)).Any())
                        throw new Exception("回报长度或内容不匹配。");
                    passed++;total++;
                    note("LINKTEST 长度="+length+" 第"+attempt+"次 PASS，耗时="+(link.Now-start)+"ms（含必要静默与往返）");
                } catch(NotSupportedException) {throw;}
                catch(IOException) {throw;}
                catch(Exception ex) {note("LINKTEST 长度="+length+" 第"+attempt+"次 FAIL："+ex.Message);}
            }
            note("LINKTEST 长度="+length+" 完整回报="+passed+"/2");
        }
        note("LINKTEST 总计="+total+"/6；原始字节及逐条结果已写入本工具logs。此次仅验证静止通信。");
        return total;
    }
    public int[] Read() {
        session.SuspendPreparation();
        Invalidate();int readEpoch=session.ConfigurationEpoch;
        string tag=Tag();int before=Info(tag);
        int[] values=new int[95];
        for(int i=0;i<values.Length;i+=8) {
            var f=Exchange("G",tag,i.ToString());
            int n=Math.Min(8,values.Length-i);
            if(f.Length!=4+n||f[3]!=i.ToString())throw new Exception("配置分页回读不匹配。");
            for(int k=0;k<n;k++)values[i+k]=int.Parse(f[4+k],CultureInfo.InvariantCulture);
        }
        CarParameterFile.Validate(values);
        if(Hash(values)!=before||Info(tag)!=before)throw new Exception("回读期间配置发生变化或数据损坏。");
        if(readEpoch!=session.ConfigurationEpoch)throw new Exception("读取期间设备重启，请重新READ。");
        verifiedValues=(int[])values.Clone();verifiedEpoch=readEpoch;
        return values;
    }
    public void Apply(int[] wanted) {
        CarParameterFile.Validate(wanted);
        int[] actual=Read();
        if(actual.SequenceEqual(wanted)){note("设备执行参数已经一致，无需改动。");return;}
        string tag=Tag();bool begun=false;int applyEpoch=session.ConfigurationEpoch;
        Invalidate();
        try {
            begun=true;
            var b=Exchange("B",tag);
            if(b.Length!=4||b[3]!=Hash(actual).ToString())throw new Exception("开始配置时设备状态已变化。");
            for(int i=0;i<wanted.Length;i++) {
                if(wanted[i]==actual[i])continue;
                var r=Exchange("S",tag,i.ToString(),wanted[i].ToString());
                if(r.Length!=5||r[3]!=i.ToString()||r[4]!=wanted[i].ToString())throw new Exception("暂存回报不匹配。");
            }
            try {
                var c=Exchange("C",tag,Hash(wanted).ToString());
                if(c.Length!=4||c[3]!=Hash(wanted).ToString())throw new Exception("提交回报不匹配。");
            } catch(TimeoutException) {
                note("提交回报未确认，自动读取实际配置核对；不重复提交。");
                if(!Read().SequenceEqual(wanted))throw new Exception("实际配置与目标不一致，本次提交未确认，未自动重发提交。");
                if(applyEpoch!=session.ConfigurationEpoch)throw new Exception("提交期间设备重启，请重新READ/APPLY。");
                begun=false;
                note("提交回报已由95项实际读回确认，参数一致。参数只在RAM中。");
                return;
            }
            begun=false;
            if(!Read().SequenceEqual(wanted))throw new Exception("提交后的完整回读不一致。");
            if(applyEpoch!=session.ConfigurationEpoch)throw new Exception("提交期间设备重启，请重新READ/APPLY。");
            note("整组提交及95项回读一致。参数只在RAM中，断电后需要重新APPLY。");
        } catch {
            Invalidate();
            if(begun&&applyEpoch==session.ConfigurationEpoch) {try{Exchange("A",tag);}catch{note("撤销未确认；不要发运动，等待READ核对或事务30秒超时。");}}
            throw;
        }
    }
}
