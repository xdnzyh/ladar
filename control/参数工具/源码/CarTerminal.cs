using System;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Drawing;
using System.Globalization;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading;
using System.Windows.Forms;
using System.Web.Script.Serialization;
using System.Text.RegularExpressions;

static class CarTerminal {
    [STAThread] public static int Main(string[] args) {
        Application.EnableVisualStyles();Application.SetCompatibleTextRenderingDefault(false);
        try {
            if(args.Length>0&&args[0]=="--test") {
                CarConfigTests.Run();CarLinkTests.Run();CarTerminalTests.Run();return 0;
            }
            if(args.Length==2&&args[0]=="--render") {
                using(var form=new CarTerminalForm(false)) {
                    form.Show();Application.DoEvents();
                    using(var bitmap=new Bitmap(form.Width,form.Height)) {
                        form.DrawToBitmap(bitmap,new Rectangle(Point.Empty,form.Size));bitmap.Save(args[1]);
                    }
                    form.Close();
                }
                return 0;
            }
            Application.Run(new CarTerminalForm(true));return 0;
        } catch(Exception ex) {if(args.Length>0){Console.Error.WriteLine(ex);return 1;}MessageBox.Show(ex.Message,"底盘终端",MessageBoxButtons.OK,MessageBoxIcon.Error);return 1;}
    }
}

// Only UI/file conversion lives here. Motion and configuration use CarSession/CarConfigClient.
static class CarTerminalModel {
    public const string Modes="WSADQEZC";
    public static readonly string[] DirectionNames={"前进 W","后退 S","左移 A","右移 D","左前 Q","右前 E","左后 Z","右后 C"};
    public static CarParameterFile Clone(CarParameterFile value) {
        return new CarParameterFile{Values=(int[])value.Values.Clone(),CountsPerMm=new Dictionary<string,decimal>(value.CountsPerMm)};
    }
    public static CarParameterFile FromCells(Func<int,string> text) {
        var value=new CarParameterFile{Values=new int[95],CountsPerMm=new Dictionary<string,decimal>()};
        for(int i=0;i<103;i++) {
            decimal number;
            if(!decimal.TryParse(text(i),NumberStyles.AllowDecimalPoint|NumberStyles.AllowLeadingSign,CultureInfo.InvariantCulture,out number))
                throw new Exception(Label(i)+"：请输入有效数字，小数点使用 .");
            if(i<95) {
                decimal scaled=number*CarParameterSchema.Scale[i];
                if(scaled!=decimal.Truncate(scaled)||scaled<CarParameterSchema.Minimum[i]||scaled>CarParameterSchema.Maximum[i])
                    throw new Exception(Label(i)+"：超出范围或小数位过多，允许范围 "+Range(i));
                value.Values[i]=(int)scaled;
            } else {
                if(number<0.0001m||number>1000m||number*10000!=decimal.Truncate(number*10000))throw new Exception(Label(i)+"：范围0.0001～1000，最多4位小数。");
                value.CountsPerMm[Modes[i-95].ToString()]=number;
            }
        }
        CarParameterFile.Validate(value.Values);return value;
    }
    public static string Value(CarParameterFile value,int i) {
        return (i<95?(decimal)value.Values[i]/CarParameterSchema.Scale[i]:value.CountsPerMm[Modes[i-95].ToString()]).ToString(CultureInfo.InvariantCulture);
    }
    public static string Range(int i) {
        if(i>=95)return "0.0001～1000 CNT/mm";
        return ((decimal)CarParameterSchema.Minimum[i]/CarParameterSchema.Scale[i]).ToString(CultureInfo.InvariantCulture)+"～"+
            ((decimal)CarParameterSchema.Maximum[i]/CarParameterSchema.Scale[i]).ToString(CultureInfo.InvariantCulture)+" "+Unit(i);
    }
    public static string Unit(int i) {
        if(i>=95)return "CNT/mm";
        if(i<44){int k=i%11;return k==8?"ms":(k==0||k==1||k==2||k==6)?"CNT/20ms":"PWM";}
        if(i<76)return "CNT";if(i<79)return "ms";if(i<81)return "%";return "";
    }
    public static string Label(int i) {
        if(i>=95)return DirectionNames[i-95]+" 距离系数";
        if(i<44) {
            string[] group={"前后移动","左右横移","斜向移动","原地旋转"};
            string[] names={"快速速度","中速速度","慢速速度","快速前馈","中速前馈","慢速前馈","起步速度","起步前馈","起步过渡时间","PWM下限","PWM上限"};
            return group[i/11]+" · "+names[i%11];
        }
        if(i<76){string[] part={"短档目标","长档目标","短档刹车提前量","长档刹车提前量"};return DirectionNames[(i-44)/4]+" · "+part[(i-44)%4];}
        string[] advanced={"运动超时","运动制动保持","空闲制动保持","中速区间占比","慢速区间占比","轮间比例增益","轮间积分增益","轮间积分输出上限","转速反馈增益","航向记忆保留系数","航向记忆增益","航向记忆上限","记忆修正上限","总航向修正上限","Y位置增益","Y速度增益","Y修正上限","S速度增益","S修正上限"};
        return advanced[i-76];
    }
    public static string Save(string path,CarParameterFile value,string expectedText) {
        if(File.ReadAllText(path,Encoding.UTF8)!=expectedText)throw new Exception("JSON已被其他程序修改。请先重新载入文件，再编辑保存。");
        string backupDir=Path.Combine(Path.GetDirectoryName(path),"backups");Directory.CreateDirectory(backupDir);
        string backup=Path.Combine(backupDir,"底盘参数_"+DateTime.Now.ToString("yyyyMMdd_HHmmss_fff")+"_"+Guid.NewGuid().ToString("N").Substring(0,6)+".json");
        string temporary=path+"."+Guid.NewGuid().ToString("N")+".tmp";
        var lines=new List<string>{"{","  \"schema\": 1,","  \"parameters\": {"};
        for(int i=0;i<95;i++)lines.Add("    \""+CarParameterSchema.Names[i]+"\": "+Value(value,i)+(i==94?"":","));
        lines.Add("  },");lines.Add("  \"counts_per_mm\": {");
        for(int i=0;i<8;i++)lines.Add("    \""+Modes[i]+"\": "+Value(value,95+i)+(i==7?"":","));
        lines.Add("  }");lines.Add("}");
        try {
            File.WriteAllLines(temporary,lines.ToArray(),new UTF8Encoding(false));
            CarParameterFile.Load(temporary); // Validate the exact file before replacement.
            if(File.ReadAllText(path,Encoding.UTF8)!=expectedText)throw new Exception("保存期间JSON已被其他程序修改，请重新载入。");
            File.Replace(temporary,path,backup);
        } finally {if(File.Exists(temporary))File.Delete(temporary);}
        return File.ReadAllText(path,Encoding.UTF8);
    }
}

// PC-only compensation: firmware continues to receive and report the wire CNT unchanged.
class RotationBrakeSettings {
    public bool Enabled=true;
    public int R=145,F=145;
    public decimal R500=20,R1000=50,F500=20,F1000=47;
    public void Validate() {
        if(R<0||R>2000||F<0||F>2000)throw new Exception("旋转提前量范围为0～2000 CNT。");
        foreach(decimal a in new[]{R500,R1000,F500,F1000})if(a<0.1m||a>360m||a*10!=decimal.Truncate(a*10))throw new Exception("标定角度范围0.1～360°，最多1位小数。");
        if(R1000<=R500||F1000<=F500)throw new Exception("1000CNT对应角度必须大于500CNT对应角度。");
    }
    public int GoalCounts(string mode,decimal degrees) {
        Validate();if(mode!="R"&&mode!="F")throw new ArgumentException("Rotation mode required");
        if(degrees<=0||degrees>360)throw new Exception("旋转角度范围大于0、至360°。");
        decimal low=mode=="R"?R500:F500,high=mode=="R"?R1000:F1000;
        // Calibration anchors are PC final-count goals, not wire counts. Lead is subtracted exactly once later.
        decimal goal=degrees<=low?500m*degrees/low:500m+500m*(degrees-low)/(high-low);
        if(goal<1||goal>100000)throw new Exception("角度换算CNT超出范围，请检查标定参数。");
        return (int)decimal.Round(goal,0,MidpointRounding.AwayFromZero);
    }
    public bool OutsideMeasuredRange(string mode,decimal degrees) {return degrees<(mode=="R"?R500:F500)||degrees>(mode=="R"?R1000:F1000);}
    public int WireCounts(string mode,int goal) {
        Validate();if(mode!="R"&&mode!="F")throw new ArgumentException("Rotation mode required");
        if(goal<1||goal>100000)throw new Exception("旋转目标范围为1～100000 CNT。");
        int wire=goal-(Enabled?(mode=="R"?R:F):0);
        if(wire<=0)throw new Exception("旋转目标必须大于提前量；请增大目标，或关闭旋转停车补偿进行原始CNT测试。");
        return wire;
    }
    public bool Same(RotationBrakeSettings b) {return Enabled==b.Enabled&&R==b.R&&F==b.F&&R500==b.R500&&R1000==b.R1000&&F500==b.F500&&F1000==b.F1000;}
    public static RotationBrakeSettings Load(string path) {
        var doc=new JavaScriptSerializer().Deserialize<Dictionary<string,object>>(File.ReadAllText(path,Encoding.UTF8));
        if(doc==null||!doc.ContainsKey("schema")||!(doc["schema"] is int)||!(((int)doc["schema"]==1&&doc.Count==4)||((int)doc["schema"]==2&&doc.Count==8))
            ||!doc.ContainsKey("enabled")||!(doc["enabled"] is bool)||!doc.ContainsKey("r_lead_cnt")||!(doc["r_lead_cnt"] is int)
            ||!doc.ContainsKey("f_lead_cnt")||!(doc["f_lead_cnt"] is int))throw new Exception("旋转停车补偿.json格式错误。");
        var value=new RotationBrakeSettings{Enabled=(bool)doc["enabled"],R=(int)doc["r_lead_cnt"],F=(int)doc["f_lead_cnt"]};
        if((int)doc["schema"]==2) {
            value.R500=Angle(doc,"r_500_deg");value.R1000=Angle(doc,"r_1000_deg");value.F500=Angle(doc,"f_500_deg");value.F1000=Angle(doc,"f_1000_deg");
        }
        value.Validate();return value;
    }
    static decimal Angle(Dictionary<string,object> doc,string key) {
        object v;if(!doc.TryGetValue(key,out v)||v==null||v is bool||v is string)throw new Exception("缺少有效标定角度："+key);
        return Convert.ToDecimal(v,CultureInfo.InvariantCulture);
    }
    public string Save(string path,string expected) {
        Validate();if(File.ReadAllText(path,Encoding.UTF8)!=expected)throw new Exception("旋转补偿文件已被外部修改，请重新载入文件。");
        string content="{\r\n  \"schema\": 2,\r\n  \"enabled\": "+(Enabled?"true":"false")+",\r\n  \"r_lead_cnt\": "+R+",\r\n  \"f_lead_cnt\": "+F
            +",\r\n  \"r_500_deg\": "+R500.ToString(CultureInfo.InvariantCulture)+",\r\n  \"r_1000_deg\": "+R1000.ToString(CultureInfo.InvariantCulture)
            +",\r\n  \"f_500_deg\": "+F500.ToString(CultureInfo.InvariantCulture)+",\r\n  \"f_1000_deg\": "+F1000.ToString(CultureInfo.InvariantCulture)+"\r\n}\r\n";
        string folder=Path.Combine(Path.GetDirectoryName(path),"backups");Directory.CreateDirectory(folder);
        string temp=path+"."+Guid.NewGuid().ToString("N")+".tmp";
        try {
            File.WriteAllText(temp,content,new UTF8Encoding(false));Load(temp);
            if(File.ReadAllText(path,Encoding.UTF8)!=expected)throw new Exception("旋转补偿文件保存期间被修改，请重新载入。");
            File.Replace(temp,path,Path.Combine(folder,"旋转停车补偿_"+DateTime.Now.ToString("yyyyMMdd_HHmmss_fff")+"_"+Guid.NewGuid().ToString("N").Substring(0,6)+".json"));
        } finally {if(File.Exists(temp))File.Delete(temp);}
        return content;
    }
    public static decimal FinalCounts(string summary) {
        var m=Regex.Match(summary??"",@"(?:^|,)ENC_CNT=(-?\d+\.\d+)(?:,|$)");
        if(!m.Success)throw new Exception("缺少已校验的旋转最终计数。");
        return decimal.Parse(m.Groups[1].Value,CultureInfo.InvariantCulture);
    }
}

class CarTerminalForm : Form {
    readonly string directory=AppDomain.CurrentDomain.BaseDirectory;
    readonly string path;
    readonly bool runWorker;
    readonly ConcurrentQueue<Action> jobs=new ConcurrentQueue<Action>();
    readonly ConcurrentQueue<string> displayLines=new ConcurrentQueue<string>();
    readonly List<Button> movement=new List<Button>();
    readonly List<Control> idleControls=new List<Control>();
    readonly List<DataGridView> grids=new List<DataGridView>();
    readonly Dictionary<int,DataGridViewRow> rows=new Dictionary<int,DataGridViewRow>();
    readonly ComboBox ports=new ComboBox();
    readonly RadioButton millimeters=new RadioButton(),encoderCounts=new RadioButton();
    readonly NumericUpDown distance=new NumericUpDown(),rotationCounts=new NumericUpDown();
    readonly NumericUpDown rotationRLead=new NumericUpDown(),rotationFLead=new NumericUpDown();
    readonly NumericUpDown[] rotationAngles={new NumericUpDown(),new NumericUpDown(),new NumericUpDown(),new NumericUpDown()};
    readonly ComboBox rotationUnit=new ComboBox();
    readonly Label rotationInputLabel=new Label();
    bool rotationWasDegrees=true;
    decimal lastDegrees=30,lastRotationCounts=500;
    readonly CheckBox rotationCompensation=new CheckBox();
    readonly Label rotationHint=new Label();
    readonly TextBox log=new TextBox();
    readonly Label stateLabel=new Label(),resultLabel=new Label(),fileLabel=new Label();
    readonly CheckBox details=new CheckBox();
    readonly Button connect=new Button(),stopButton=new Button(),confirmButton=new Button();
    readonly System.Windows.Forms.Timer timer=new System.Windows.Forms.Timer();
    Thread worker;
    ICarLink link;
    CarSession session;
    CarConfigClient client;
    CarParameterFile config;
    string diskText;
    readonly string rotationPath;
    string rotationDiskText;
    RotationBrakeSettings rotationBrake;
    StreamWriter events;
    FileStream raw;
    volatile bool connected,busy,dirty,stopRequested,closing,verified,uncertain,unresolved;
    bool filling;
    volatile string operation="未连接",outcome="连接后即可运动；需要同步时点击读取或应用参数。";
    int[] actual;
    int seenEpoch=-1;
    public CarTerminalForm(bool enableWorker) {
        runWorker=enableWorker;path=Path.Combine(directory,"底盘参数.json");
        diskText=File.ReadAllText(path,Encoding.UTF8);config=CarParameterFile.Load(path);
        rotationPath=Path.Combine(directory,"旋转停车补偿.json");
        rotationDiskText=File.ReadAllText(rotationPath,Encoding.UTF8);rotationBrake=RotationBrakeSettings.Load(rotationPath);
        Text="底盘集成终端 · 参数与运动";Size=new Size(1220,850);MinimumSize=new Size(1080,740);
        StartPosition=FormStartPosition.CenterScreen;Font=new Font("Microsoft YaHei UI",9F);
        BackColor=Color.FromArgb(242,245,249);KeyPreview=true;
        Build();Fill(config);FillRotation();RefreshPorts();
        timer.Interval=80;timer.Tick+=(s,e)=>RefreshState();timer.Start();
        Shown+=(s,e)=>{if(runWorker){worker=new Thread(Work){IsBackground=true,Name="CarSerial"};worker.Start();}};
        FormClosing+=OnClosing;
        KeyDown+=(s,e)=>{if(e.KeyCode==Keys.Escape){RequestStop();e.Handled=true;e.SuppressKeyPress=true;}};
        RefreshState();
    }
    Button Button(string text,Action action,Color? color=null) {
        var b=new Button{Text=text,Height=35,AutoSize=false,Width=150,FlatStyle=FlatStyle.Flat,BackColor=color??Color.White,Margin=new Padding(4)};
        b.FlatAppearance.BorderColor=Color.FromArgb(205,213,224);b.Click+=(s,e)=>{try{action();}catch(Exception ex){outcome=ex.Message;MessageBox.Show(this,ex.Message,"操作未执行",MessageBoxButtons.OK,MessageBoxIcon.Information);}};
        return b;
    }
    Label Caption(string text) {return new Label{Text=text,AutoSize=true,Font=new Font(Font,FontStyle.Bold),Margin=new Padding(4,10,4,7)};}
    void Build() {
        var root=new TableLayoutPanel{Dock=DockStyle.Fill,ColumnCount=1,RowCount=4,Padding=new Padding(12)};
        root.RowStyles.Add(new RowStyle(SizeType.Absolute,66));root.RowStyles.Add(new RowStyle(SizeType.Percent,100));
        root.RowStyles.Add(new RowStyle(SizeType.Absolute,160));root.RowStyles.Add(new RowStyle(SizeType.Absolute,30));Controls.Add(root);
        var header=new FlowLayoutPanel{Dock=DockStyle.Fill,WrapContents=false};
        header.Controls.Add(new Label{Text="底盘集成终端",AutoSize=true,Font=new Font(Font.FontFamily,18,FontStyle.Bold),Margin=new Padding(3,12,30,3)});
        ports.DropDownStyle=ComboBoxStyle.DropDownList;ports.Width=100;ports.Margin=new Padding(4,18,4,4);header.Controls.Add(ports);idleControls.Add(ports);
        var refresh=Button("刷新串口",RefreshPorts);refresh.Width=88;refresh.Margin=new Padding(4,12,4,4);header.Controls.Add(refresh);idleControls.Add(refresh);
        connect.Text="连接";connect.Size=new Size(122,35);connect.Margin=new Padding(4,12,20,4);connect.Click+=(s,e)=>Connect();header.Controls.Add(connect);
        stateLabel.AutoSize=true;stateLabel.Margin=new Padding(4,21,4,4);header.Controls.Add(stateLabel);root.Controls.Add(header,0,0);
        var body=new TableLayoutPanel{Dock=DockStyle.Fill,ColumnCount=2,RowCount=1};body.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute,320));body.ColumnStyles.Add(new ColumnStyle(SizeType.Percent,100));root.Controls.Add(body,0,1);
        var left=new FlowLayoutPanel{Dock=DockStyle.Fill,FlowDirection=FlowDirection.TopDown,WrapContents=false,AutoScroll=true,BackColor=Color.White,Padding=new Padding(10)};body.Controls.Add(left,0,0);
        left.Controls.Add(Caption("运动测试"));
        left.Controls.Add(new Label{Text="点击一次，执行一个定距离动作",AutoSize=true,ForeColor=Color.DimGray});
        var amount=new FlowLayoutPanel{Size=new Size(274,45),WrapContents=false,Margin=new Padding(0,10,0,4)};
        distance.Minimum=1;distance.Maximum=100000;distance.Value=100;distance.Width=115;distance.Height=30;amount.Controls.Add(distance);
        millimeters.Text="毫米";millimeters.Checked=true;millimeters.Size=new Size(67,27);
        encoderCounts.Text="CNT";encoderCounts.Size=new Size(67,27);
        amount.Controls.Add(millimeters);amount.Controls.Add(encoderCounts);left.Controls.Add(amount);idleControls.Add(distance);idleControls.Add(millimeters);idleControls.Add(encoderCounts);
        var pad=new TableLayoutPanel{Size=new Size(274,186),ColumnCount=3,RowCount=3,Margin=new Padding(0,2,0,4)};
        for(int i=0;i<3;i++){pad.ColumnStyles.Add(new ColumnStyle(SizeType.Percent,33.33F));pad.RowStyles.Add(new RowStyle(SizeType.Percent,33.33F));}
        string[] modes={"Q","W","E","A","","D","Z","S","C"};string[] names={"↖ 左前 Q","↑ 前进 W","↗ 右前 E","← 左移 A","停止\nEsc","右移 D →","↙ 左后 Z","↓ 后退 S","↘ 右后 C"};
        for(int i=0;i<9;i++) {
            Button b;
            if(i==4){b=stopButton;b.Text=names[i];b.BackColor=Color.FromArgb(199,49,58);b.ForeColor=Color.White;b.Click+=(s,e)=>RequestStop();}
            else {string mode=modes[i];b=Button(names[i],()=>SendMove(mode));movement.Add(b);}
            b.Dock=DockStyle.Fill;b.Margin=new Padding(3);pad.Controls.Add(b,i%3,i/3);
        }
        left.Controls.Add(pad);
        var rotateAmount=new FlowLayoutPanel{Size=new Size(274,33),WrapContents=false,Margin=new Padding(0,6,0,0)};
        rotationInputLabel.Text="旋转角度(°)";rotationInputLabel.AutoSize=true;rotationInputLabel.Margin=new Padding(3,6,4,0);rotateAmount.Controls.Add(rotationInputLabel);
        rotationCounts.Minimum=0.1m;rotationCounts.Maximum=360;rotationCounts.DecimalPlaces=1;rotationCounts.Value=30;rotationCounts.Width=96;
        rotateAmount.Controls.Add(rotationCounts);left.Controls.Add(rotateAmount);idleControls.Add(rotationCounts);
        rotationUnit.DropDownStyle=ComboBoxStyle.DropDownList;rotationUnit.Items.AddRange(new object[]{"度 (°)","CNT"});rotationUnit.Width=70;rotationUnit.SelectedIndex=0;
        rotationUnit.SelectedIndexChanged+=(s,e)=>SwitchRotationUnit();rotateAmount.Controls.Add(rotationUnit);idleControls.Add(rotationUnit);
        var rotatePad=new FlowLayoutPanel{Size=new Size(274,43),WrapContents=false,Margin=new Padding(0)};
        foreach(string mode in new[]{"F","R"}) {
            string direction=mode;var b=Button(mode=="F"?"↶ 逆时针 F":"顺时针 R ↷",()=>SendMove(direction));
            b.Width=128;rotatePad.Controls.Add(b);movement.Add(b);
        }
        left.Controls.Add(rotatePad);
        rotationHint.Size=new Size(274,44);rotationHint.ForeColor=Color.DimGray;rotationHint.Margin=new Padding(3,0,0,6);left.Controls.Add(rotationHint);
        confirmButton.Text="已确认车停稳 · 解锁";confirmButton.Size=new Size(268,35);confirmButton.Click+=(s,e)=>Enqueue("确认停止",()=>{stopRequested=false;session.Confirm();unresolved=false;outcome="已按现场确认解锁。";});left.Controls.Add(confirmButton);
        left.Controls.Add(Caption("本次状态"));resultLabel.Size=new Size(268,58);resultLabel.ForeColor=Color.FromArgb(37,63,88);left.Controls.Add(resultLabel);
        var ping=Button("检查连接",()=>Enqueue("检查连接",()=>{outcome=session.Ping(5)?"连接回应正常。":"握手未完成。";}));ping.Width=268;left.Controls.Add(ping);idleControls.Add(ping);ping.Tag="connected";
        var editor=new TableLayoutPanel{Dock=DockStyle.Fill,ColumnCount=1,RowCount=4,Padding=new Padding(12,0,0,0)};
        editor.RowStyles.Add(new RowStyle(SizeType.Absolute,53));editor.RowStyles.Add(new RowStyle(SizeType.Absolute,32));editor.RowStyles.Add(new RowStyle(SizeType.Percent,100));editor.RowStyles.Add(new RowStyle(SizeType.Absolute,47));body.Controls.Add(editor,1,0);
        var buttons=new FlowLayoutPanel{Dock=DockStyle.Fill,WrapContents=false};
        var read=Button("读取车上参数",()=>Enqueue("读取参数",ReadDevice));read.Tag="connected";
        var save=Button("保存到电脑",()=>Save(false));
        var apply=Button("保存并应用到小车",()=>Save(true),Color.FromArgb(224,238,252));apply.Width=174;apply.Tag="connected";
        var reload=Button("重新载入文件",Reload);reload.Width=126;
        foreach(var b in new[]{read,save,apply,reload}){buttons.Controls.Add(b);idleControls.Add(b);}editor.Controls.Add(buttons,0,0);
        editor.Controls.Add(new Label{Text="距离系数、旋转补偿：保存到电脑即生效。车端参数：保存并应用到小车。",Dock=DockStyle.Fill,ForeColor=Color.DimGray},0,1);
        var tabs=new TabControl{Dock=DockStyle.Fill};string[] groups={"距离标定","运动参数","刹车补偿","高级参数"};
        for(int tab=0;tab<4;tab++) {
            var page=new TabPage(groups[tab]);var grid=new DataGridView{Dock=DockStyle.Fill,AllowUserToAddRows=false,AllowUserToDeleteRows=false,AllowUserToOrderColumns=false,RowHeadersVisible=false,BackgroundColor=Color.White,BorderStyle=BorderStyle.None,AutoSizeColumnsMode=DataGridViewAutoSizeColumnsMode.Fill,SelectionMode=DataGridViewSelectionMode.CellSelect,MultiSelect=false,EditMode=DataGridViewEditMode.EditOnEnter};
            grid.Columns.Add("label","参数");grid.Columns.Add("edit","编辑值");grid.Columns.Add("device","设备读回值");grid.Columns.Add("range","允许范围 / 单位");
            grid.Columns[0].FillWeight=40;grid.Columns[1].FillWeight=16;grid.Columns[2].FillWeight=17;grid.Columns[3].FillWeight=27;
            for(int c=0;c<4;c++)grid.Columns[c].ReadOnly=c!=1;
            foreach(DataGridViewColumn c in grid.Columns)c.SortMode=DataGridViewColumnSortMode.NotSortable;
            grid.RowTemplate.Height=31;grid.ColumnHeadersHeight=34;grid.EnableHeadersVisualStyles=false;grid.ColumnHeadersDefaultCellStyle.BackColor=Color.FromArgb(233,239,246);
            for(int i=0;i<103;i++) {
                int group=i>=95?0:i<44?1:i>=44&&i<76?2:3;if(group!=tab)continue;
                int row=grid.Rows.Add(CarTerminalModel.Label(i),"",i>=95?"电脑端":"未读取",CarTerminalModel.Range(i));
                rows[i]=grid.Rows[row];rows[i].Tag=i;rows[i].Cells[0].ToolTipText=i<95?CarParameterSchema.Names[i]:"counts_per_mm."+CarTerminalModel.Modes[i-95];
            }
            grid.CellBeginEdit+=(s,e)=>{if(!filling){dirty=true;outcome="编辑中：请保存，或保存并应用。";}};
            grid.CellValueChanged+=(s,e)=>{if(!filling&&e.RowIndex>=0&&e.ColumnIndex==1)dirty=true;};
            grid.DataError+=(s,e)=>{e.ThrowException=false;outcome="请输入有效数值。";};
            grids.Add(grid);page.Controls.Add(grid);tabs.TabPages.Add(page);
        }
        var rotationPage=new TabPage("旋转标定/补偿");
        var rotationPanel=new FlowLayoutPanel{Dock=DockStyle.Fill,FlowDirection=FlowDirection.TopDown,WrapContents=false,Padding=new Padding(16),AutoScroll=true};
        rotationCompensation.Text="启用旋转停车补偿（电脑端）";rotationCompensation.AutoSize=true;rotationPanel.Controls.Add(rotationCompensation);idleControls.Add(rotationCompensation);
        foreach(bool clockwise in new[]{true,false}) {
            var line=new FlowLayoutPanel{Width=400,Height=42,WrapContents=false};
            line.Controls.Add(new Label{Text=clockwise?"R顺时针提前量（CNT）":"F逆时针提前量（CNT）",Width=195,Margin=new Padding(3,7,3,0)});
            var input=clockwise?rotationRLead:rotationFLead;input.Minimum=0;input.Maximum=2000;input.Width=110;line.Controls.Add(input);rotationPanel.Controls.Add(line);idleControls.Add(input);
            input.ValueChanged+=(s,e)=>{if(!filling){dirty=true;outcome="旋转补偿已编辑，点击保存到电脑后生效。";}};
        }
        rotationCompensation.CheckedChanged+=(s,e)=>{if(!filling){dirty=true;outcome="旋转补偿已编辑，点击保存到电脑后生效。";}};
        string[] angleLabels={"R终点500CNT实测角度","R终点1000CNT实测角度","F终点500CNT实测角度","F终点1000CNT实测角度"};
        for(int i=0;i<4;i++) {
            var line=new FlowLayoutPanel{Width=420,Height=37,WrapContents=false};line.Controls.Add(new Label{Text=angleLabels[i]+"（°）",Width=225,Margin=new Padding(3,7,3,0)});
            var input=rotationAngles[i];input.Minimum=0.1m;input.Maximum=360;input.DecimalPlaces=1;input.Width=100;line.Controls.Add(input);rotationPanel.Controls.Add(line);idleControls.Add(input);
            input.ValueChanged+=(s,e)=>{if(!filling){dirty=true;outcome="旋转角度已编辑，点击保存到电脑后生效。";}};
        }
        rotationPanel.Controls.Add(new Label{Text="角度先换算为终点CNT，再减提前量一次。\n两档之间线性插值，较大角度沿用该斜率；低于小档按比例换算。\n以上角度来自补偿开启后的实测，未测范围为估算。\n修改后保存到电脑即可，重开终端仍保留。\n关闭补偿只供CNT原始测试；度控制需开启补偿。\n本页不改变旋转动力，车端重启后仍需按需应用动力参数。",AutoSize=true,Margin=new Padding(3,12,3,3)});
        rotationPage.Controls.Add(rotationPanel);tabs.TabPages.Add(rotationPage);editor.Controls.Add(tabs,0,2);
        var footer=new FlowLayoutPanel{Dock=DockStyle.Fill,WrapContents=false};
        var export=Button("导出设备配置",Export);export.Tag="connected";footer.Controls.Add(export);idleControls.Add(export);
        footer.Controls.Add(new Label{Text="车端参数断电恢复默认，重新开电后应用电脑配置。",AutoSize=true,Margin=new Padding(8,12,0,0),ForeColor=Color.DimGray});editor.Controls.Add(footer,0,3);
        var logArea=new TableLayoutPanel{Dock=DockStyle.Fill,RowCount=2,ColumnCount=1,Padding=new Padding(0,8,0,0)};logArea.RowStyles.Add(new RowStyle(SizeType.Absolute,28));logArea.RowStyles.Add(new RowStyle(SizeType.Percent,100));
        var logBar=new FlowLayoutPanel{Dock=DockStyle.Fill};logBar.Controls.Add(new Label{Text="操作记录",AutoSize=true,Font=new Font(Font,FontStyle.Bold),Margin=new Padding(2,3,20,0)});
        details.Text="显示通信明细";details.AutoSize=true;logBar.Controls.Add(details);
        var open=Button("打开日志文件夹",()=>OpenFolder(Path.Combine(directory,"logs")));open.Size=new Size(132,25);open.Margin=new Padding(20,0,0,0);logBar.Controls.Add(open);logArea.Controls.Add(logBar,0,0);
        log.Multiline=true;log.ReadOnly=true;log.ScrollBars=ScrollBars.Vertical;log.Dock=DockStyle.Fill;log.BackColor=Color.FromArgb(248,250,253);log.Font=new Font("Consolas",9);logArea.Controls.Add(log,0,1);root.Controls.Add(logArea,0,2);
        fileLabel.Text="配置："+path;fileLabel.Dock=DockStyle.Fill;fileLabel.AutoEllipsis=true;fileLabel.ForeColor=Color.DimGray;root.Controls.Add(fileLabel,0,3);
    }
    void OpenFolder(string folder) {Directory.CreateDirectory(folder);System.Diagnostics.Process.Start(new System.Diagnostics.ProcessStartInfo(folder){UseShellExecute=true});}
    void Fill(CarParameterFile value) {
        filling=true;try {foreach(var row in rows){row.Value.Cells[1].Value=CarTerminalModel.Value(value,row.Key);}}finally{filling=false;}dirty=false;
    }
    void FillRotation() {
        filling=true;try{rotationRLead.Value=rotationBrake.R;rotationFLead.Value=rotationBrake.F;rotationCompensation.Checked=rotationBrake.Enabled;
            decimal[] angles={rotationBrake.R500,rotationBrake.R1000,rotationBrake.F500,rotationBrake.F1000};for(int i=0;i<4;i++)rotationAngles[i].Value=angles[i];
        }finally{filling=false;}
        rotationHint.Text=rotationBrake.Enabled?"R提前"+rotationBrake.R+"，F提前"+rotationBrake.F+" CNT\n度控制已接入，未测角度按标定外推。":"补偿关闭：CNT原始测试可用。\n度控制请先开启补偿并保存。";
    }
    void SwitchRotationUnit() {
        if(rotationWasDegrees)lastDegrees=rotationCounts.Value;else lastRotationCounts=rotationCounts.Value;
        rotationWasDegrees=rotationUnit.SelectedIndex==0;
        rotationInputLabel.Text=rotationWasDegrees?"旋转角度(°)":"终点(CNT)";
        rotationCounts.Minimum=rotationWasDegrees?0.1m:1;rotationCounts.Maximum=rotationWasDegrees?360:100000;rotationCounts.DecimalPlaces=rotationWasDegrees?1:0;
        rotationCounts.Value=rotationWasDegrees?lastDegrees:lastRotationCounts;
    }
    CarParameterFile Draft() {foreach(var grid in grids)grid.EndEdit();return CarTerminalModel.FromCells(i=>Convert.ToString(rows[i].Cells[1].Value,CultureInfo.InvariantCulture));}
    void RefreshPorts() {string selected=Convert.ToString(ports.SelectedItem);ports.Items.Clear();ports.Items.AddRange(SerialPort.GetPortNames().OrderBy(s=>s).ToArray());if(ports.Items.Contains(selected))ports.SelectedItem=selected;else if(ports.Items.Count>0)ports.SelectedIndex=0;}
    void Ui(Action action) {if(IsDisposed||closing)return;try{BeginInvoke(action);}catch(InvalidOperationException){}}
    void Note(string text) {if(events!=null){events.WriteLine(DateTime.Now.ToString("O")+" "+text);events.Flush();}displayLines.Enqueue(text);}
    void Enqueue(string title,Action action) {
        if(busy||closing)return;
        if(runWorker&&worker!=null&&!worker.IsAlive){outcome="后台已退出，请关闭并重新打开终端。";return;}
        busy=true;operation=title;RefreshState();
        jobs.Enqueue(()=>{try{action();}catch(Exception ex){outcome="未确认成功："+ex.Message;Note(outcome);if(ex is IOException||ex is UnauthorizedAccessException)Disconnect();}
            finally{Snapshot();busy=false;operation=connected?"已连接":"未连接";}});
    }
    void Connect() {
        if(busy)return;
        if(connected){Enqueue("断开连接",Disconnect);return;}
        string port=Convert.ToString(ports.SelectedItem);if(string.IsNullOrWhiteSpace(port)){outcome="没有选中串口，请刷新后选择。";return;}
        Enqueue("连接",()=>{
            AttachLink(new SerialCarLink(port),s=>{byte[] b=Encoding.GetEncoding(28591).GetBytes(s);raw.Write(b,0,b.Length);raw.Flush();});
            Note("已连接端口："+port);outcome="已连接，使用车上现有参数。需要同步时点击读取或应用。";
        });
    }
    void AttachLink(ICarLink transport,Action<string> receive) {
        link=transport;session=new CarSession(link,Note,receive,()=>Guid.NewGuid().ToString("N").Substring(0,8).ToUpperInvariant());
        session.Padding=32;client=new CarConfigClient(link,session,Note);
        // Configuration readback is optional; motion completion/stop interlocks remain mandatory.
        session.BeforeMove=()=>!dirty&&!unresolved&&!stopRequested;
        session.EmergencyRequested=()=>stopRequested;connected=true;seenEpoch=session.ConfigurationEpoch;stopRequested=false;
        actual=null;verified=false;
    }
    void Disconnect() {
        if(session!=null&&session.Uncertain)unresolved=true;
        if(link!=null){var disposable=link as IDisposable;if(disposable!=null)disposable.Dispose();link=null;}connected=false;verified=false;session=null;client=null;actual=null;
        Ui(()=>ShowActual(null));Note("连接已断开。");
    }
    void ReadDevice() {
        int[] values=client.Read();actual=(int[])values.Clone();Ui(()=>ShowActual(values));
        int count=values.Where((v,i)=>v!=config.Values[i]).Count();
        outcome=count==0?"读取成功，车上参数与电脑一致。":"读取成功：有 "+count+" 项与电脑不同，点击保存并应用可同步。";
        Note(outcome+" CRC="+CarConfigClient.Hash(values));
    }
    void ShowActual(int[] values) {
        for(int i=0;i<95;i++) {
            rows[i].Cells[2].Value=values==null?"未读取":((decimal)values[i]/CarParameterSchema.Scale[i]).ToString(CultureInfo.InvariantCulture);
            rows[i].Cells[2].Style.BackColor=values!=null&&values[i]!=config.Values[i]?Color.FromArgb(255,239,204):Color.White;
        }
    }
    void Save(bool apply) {
        if(busy)return;CarParameterFile draft=Draft();
        var rotationDraft=new RotationBrakeSettings{Enabled=rotationCompensation.Checked,R=(int)rotationRLead.Value,F=(int)rotationFLead.Value,
            R500=rotationAngles[0].Value,R1000=rotationAngles[1].Value,F500=rotationAngles[2].Value,F1000=rotationAngles[3].Value};rotationDraft.Validate();
        if(apply&&(!connected||uncertain))throw new Exception(uncertain?"请先确认车已停稳并解锁。":"请先连接小车。");
        Enqueue(apply?"保存并应用参数":"保存参数",()=>{
            if(File.ReadAllText(path,Encoding.UTF8)!=diskText)throw new Exception("文件已被其他程序修改，请先重新载入。");
            if(File.ReadAllText(rotationPath,Encoding.UTF8)!=rotationDiskText)throw new Exception("旋转补偿文件已被其他程序修改，请先重新载入。");
            bool valuesChanged=!config.Values.SequenceEqual(draft.Values);
            bool rotationChanged=!rotationBrake.Same(rotationDraft);
            bool changed=valuesChanged||CarTerminalModel.Modes.Any(m=>config.CountsPerMm[m.ToString()]!=draft.CountsPerMm[m.ToString()]);
            if(changed)diskText=CarTerminalModel.Save(path,draft,diskText);
            if(rotationChanged)rotationDiskText=rotationDraft.Save(rotationPath,rotationDiskText);
            rotationBrake=rotationDraft;Ui(FillRotation);
            config=CarTerminalModel.Clone(draft);dirty=false;
            if(connected){session.SuspendPreparation();if(valuesChanged)client.Invalidate();}
            Note(changed?"已保存电脑配置，原文件已自动备份。":"电脑配置未变化。");
            outcome="已保存电脑配置。";
            if(apply){client.Apply(config.Values);actual=(int[])config.Values.Clone();Ui(()=>ShowActual(actual));outcome="保存并应用成功，车上参数完整读回一致。";Note(outcome);}
            else if(valuesChanged)outcome="已保存到电脑，车辆继续使用现有参数；点击应用时更新。";
            else outcome="已保存，距离系数和旋转补偿已在电脑端生效。";
            if(rotationChanged)Note("旋转补偿已保存：ENABLED="+rotationBrake.Enabled+", R_LEAD="+rotationBrake.R+", F_LEAD="+rotationBrake.F+"；未向车端写入补偿参数。");
        });
    }
    void Reload() {
        if(dirty&&MessageBox.Show(this,"放弃窗口中尚未保存的编辑，重新读取JSON？","重新载入",MessageBoxButtons.YesNo,MessageBoxIcon.Question)!=DialogResult.Yes)return;
        Enqueue("重新载入文件",()=>{
            string text=File.ReadAllText(path,Encoding.UTF8);var loaded=CarParameterFile.Load(path);
            string rotationText=File.ReadAllText(rotationPath,Encoding.UTF8);var rotationLoaded=RotationBrakeSettings.Load(rotationPath);
            if(connected){session.SuspendPreparation();if(!loaded.Values.SequenceEqual(config.Values))client.Invalidate();}
            config=loaded;diskText=text;rotationBrake=rotationLoaded;rotationDiskText=rotationText;Ui(()=>{Fill(loaded);FillRotation();ShowActual(actual);});outcome="已重新载入电脑配置和旋转补偿。";Note(outcome);
        });
    }
    void Export() {
        using(var dialog=new SaveFileDialog{Filter="JSON配置|*.json",FileName="底盘参数_导出_"+DateTime.Now.ToString("yyyyMMdd_HHmmss")+".json",InitialDirectory=directory}) {
            if(dialog.ShowDialog(this)!=DialogResult.OK)return;string destination=dialog.FileName;
            if(string.Equals(Path.GetFullPath(destination),Path.GetFullPath(path),StringComparison.OrdinalIgnoreCase))throw new Exception("导出请使用新文件名，避免覆盖当前配置。");
            Enqueue("导出设备配置",()=>{ReadDevice();config.Save(destination,actual);outcome="已导出设备参数，距离系数沿用电脑配置。";Note(outcome+" "+destination);});
        }
    }
    void SendMove(string mode) {
        if(!connected||busy||dirty||uncertain||stopRequested)return;
        bool rotation=mode=="R"||mode=="F";
        bool degrees=rotation&&rotationUnit.SelectedIndex==0;
        decimal value=rotation?rotationCounts.Value:distance.Value;string unit=rotation?(degrees?"DEG":"CNT"):millimeters.Checked?"MM":"CNT";
        Enqueue("执行 "+mode+" "+value+" "+unit,()=>{
            if(stopRequested||session.Uncertain||dirty||unresolved)throw new Exception("请先保存编辑或确认车辆停止。");
            if(degrees&&!rotationBrake.Enabled)throw new Exception("度控制使用补偿后的标定，请开启旋转补偿并保存，或切换CNT原始测试。");
            int goal=degrees?rotationBrake.GoalCounts(mode,value):(int)value;
            long counts=rotation?rotationBrake.WireCounts(mode,goal):unit=="MM"?config.ToCounts(mode,(int)value):(int)value;
            bool deviceKnown=actual!=null&&client.IsVerified(actual);
            Note("请求="+mode+" "+value+" "+unit+"，发送CNT="+counts+", FILE_CRC="+CarConfigClient.Hash(config.Values)+", DEVICE_CRC="+(deviceKnown?CarConfigClient.Hash(actual).ToString():"未核对"));
            if(rotation)Note("ROTATION_REQUEST,MODE="+mode+",INPUT="+value.ToString(CultureInfo.InvariantCulture)+",UNIT="+unit+",GOAL_CNT="+goal+",WIRE_CNT="+counts+",LEAD="+(goal-counts)+",COMPENSATED="+(rotationBrake.Enabled?"1":"0"));
            if(degrees)Note("ROTATION_ANGLE,MODE="+mode+",REQ_DEG="+value.ToString(CultureInfo.InvariantCulture)+",CAL500_DEG="+(mode=="R"?rotationBrake.R500:rotationBrake.F500).ToString(CultureInfo.InvariantCulture)+",CAL1000_DEG="+(mode=="R"?rotationBrake.R1000:rotationBrake.F1000).ToString(CultureInfo.InvariantCulture)+",EXTRAPOLATED="+(rotationBrake.OutsideMeasuredRange(mode,value)?"1":"0"));
            // An unread device may use a longer timeout than the local file. This is only a deadline, not a delay.
            int deadline=deviceKnown?Math.Min(21000,Math.Max(14000,actual[76]+6000)):21000;
            string result=session.MoveRecoverable(mode,counts.ToString(CultureInfo.InvariantCulture),"CNT",deadline);
            outcome=result=="TARGET"?mode+" "+value+" "+unit+" 已完成，结果校验通过。":"动作结果："+result+"。请查看操作记录。";
            if(rotation&&result=="TARGET") {
                decimal enc=RotationBrakeSettings.FinalCounts(session.LastResultSummary),error=enc-goal;
                Note("ROTATION_STOP,MODE="+mode+",INPUT="+value.ToString(CultureInfo.InvariantCulture)+",UNIT="+unit+",GOAL_CNT="+goal+",WIRE_CNT="+counts+",LEAD="+(goal-counts)+",ENC_CNT="+enc.ToString("F2",CultureInfo.InvariantCulture)+",STOP_ERROR="+error.ToString("F2",CultureInfo.InvariantCulture));
                outcome=degrees?mode+" "+value+"°指令已完成，实际角度需测量。\n编码器停车误差 "+error.ToString("+0.00;-0.00;0.00",CultureInfo.InvariantCulture)+" CNT。":
                    mode+" 已完成：终点目标"+goal+"，实际"+enc.ToString("F2",CultureInfo.InvariantCulture)+" CNT\n停车误差 "+error.ToString("+0.00;-0.00;0.00",CultureInfo.InvariantCulture)+" CNT（实际−目标）。";
            }
        });
    }
    void RequestStop() {if(!connected)return;stopRequested=true;outcome="已请求停止，等待车辆回报。";RefreshState();}
    void Snapshot() {
        verified=connected&&client!=null&&client.IsVerified(config.Values);uncertain=unresolved||(connected&&session!=null&&session.Uncertain);
        if(session!=null&&seenEpoch!=session.ConfigurationEpoch){seenEpoch=session.ConfigurationEpoch;actual=null;Ui(()=>ShowActual(null));}
    }
    void Work() {
        try {
            string logs=Path.Combine(directory,"logs");Directory.CreateDirectory(logs);string stamp=DateTime.Now.ToString("yyyyMMdd_HHmmss_fff");
            events=new StreamWriter(Path.Combine(logs,stamp+"_terminal.txt"),false,new UTF8Encoding(false));raw=new FileStream(Path.Combine(logs,stamp+"_terminal_rx.bin"),FileMode.CreateNew,FileAccess.Write);
            Note("TERMINAL=1 ROTATION_CNT=1 ROTATION_BRAKE=1 ROTATION_DEG=1 MANUAL_CONFIG_SYNC=1 CONFIG_RECOVERY=1 RESULT_RECOVERY=1 IDLE_PREFLIGHT=1");Note("配置文件："+path);
            Note("旋转补偿文件："+rotationPath+" ENABLED="+rotationBrake.Enabled+", R_LEAD="+rotationBrake.R+", F_LEAD="+rotationBrake.F);
            while(!closing) {
                Action job;if(jobs.TryDequeue(out job)){job();continue;}
                if(connected) {
                    try{session.Poll();session.PrepareNextMove();Snapshot();}
                    catch(Exception ex){outcome="通信中断："+ex.Message;Note(outcome);Disconnect();}
                }
                Thread.Sleep(15);
            }
        } catch(Exception ex){outcome="终端后台退出："+ex.Message;Ui(()=>MessageBox.Show(this,outcome));}
        finally {connected=false;verified=false;busy=false;var disposable=link as IDisposable;if(disposable!=null)disposable.Dispose();if(raw!=null)raw.Dispose();if(events!=null)events.Dispose();}
    }
    void RefreshState() {
        string line;while(displayLines.TryDequeue(out line)) {
            bool wire=line.StartsWith("RX ")||line.StartsWith("TX ")||line.StartsWith("RESULT_OK,");
            if(details.Checked||!wire)log.AppendText(DateTime.Now.ToString("HH:mm:ss")+"  "+line+Environment.NewLine);
        }
        if(log.TextLength>45000)log.Text=log.Text.Substring(log.TextLength-30000);
        stateLabel.Text=busy?operation:!connected?"未连接":uncertain?"待确认停止":dirty?"有未保存修改":verified?"参数已核对 · 可以运动":"已连接 · 使用车上现有参数";
        stateLabel.ForeColor=connected&&!uncertain&&!dirty?Color.FromArgb(27,125,86):Color.FromArgb(123,78,27);
        resultLabel.Text=outcome;connect.Text=connected?"断开连接":"连接";connect.Enabled=!busy;
        foreach(var control in idleControls)control.Enabled=!busy&&(control.Tag==null||connected);
        ports.Enabled=!connected&&!busy;foreach(var grid in grids)grid.Enabled=!busy;
        foreach(var b in movement)b.Enabled=connected&&!busy&&!dirty&&!uncertain&&!stopRequested;
        stopButton.Enabled=connected;confirmButton.Enabled=connected&&!busy&&(uncertain||stopRequested);
    }
    void OnClosing(object sender,FormClosingEventArgs e) {
        if(busy){e.Cancel=true;outcome="操作进行中，请停止或等待结束后关闭。";return;}
        if(dirty&&MessageBox.Show(this,"有未保存的参数编辑，仍然退出？","退出终端",MessageBoxButtons.YesNo,MessageBoxIcon.Question)!=DialogResult.Yes){e.Cancel=true;return;}
        if(connected&&uncertain&&MessageBox.Show(this,"上一动作尚未确认。请确认车辆已停稳后再关闭。现在退出？","退出终端",MessageBoxButtons.YesNo,MessageBoxIcon.Question)!=DialogResult.Yes){e.Cancel=true;return;}
        closing=true;timer.Stop();if(worker!=null)worker.Join(1800);
    }
    internal void OfflineChecks() {
        timer.Stop();var fake=new FakeCarLink();bool stopOnMove=false,dropConfig=false,dropRotationResult=false;string resultBody=null;
        int[] deviceValues=(int[])config.Values.Clone();
        if(movement.Count!=10||movement.Any(b=>b.Enabled)||rotationCounts.Value!=30||rotationUnit.SelectedIndex!=0)throw new Exception("Rotation controls not initially locked/defaulted");
        rotationBrake=new RotationBrakeSettings();FillRotation();
        rotationUnit.SelectedIndex=1;
        if(rotationCounts.Value!=500)throw new Exception("Unit switch lost independent CNT input");
        fake.OnWrite=(wire,f)=>{
            string text=wire.Trim();
            if(text.StartsWith("@PING,")){f.Later(20,text.Replace("@PING,","@PONG,")+"\r\n");return;}
            if(text.StartsWith("@MOVE,")) {
                var parts=text.Split(',');string count=parts[2];
                string mode=parts[1];if(mode!="W"&&mode!="R"&&mode!="F")throw new Exception("Unexpected offline motion");
                if(parts[3]!="CNT")throw new Exception("UI sent non-CNT wire request");
                string final=(int.Parse(count)+(mode=="W"?0:145)).ToString(CultureInfo.InvariantCulture);
                string wheels=mode=="W"?string.Join(",",Enumerable.Repeat(final,4)):
                    mode=="R"?"-"+final+","+final+","+final+",-"+final:final+",-"+final+",-"+final+","+final;
                resultBody="@RESULT,"+parts[4]+","+mode+","+(stopOnMove?"1":"0")+","+count+",CNT,"+count+".00,"+final+".00,"+wheels;
                if(stopOnMove)RequestStop();else if(!(dropRotationResult&&mode!="W"))f.Later(20,CarSession.CheckedCommand(resultBody)+"\r\n");return;
            }
            if(text.StartsWith("@RESULT,")){f.Later(20,CarSession.CheckedCommand(resultBody)+"\r\n");return;}
            if(wire=="!\r\nX\r\n"){f.Later(20,CarSession.CheckedCommand(resultBody)+"\r\n");return;}
            var q=CarConfigClient.Parse(text);if(q==null)throw new Exception("Unexpected test frame");
            if(dropConfig)return;
            string reply="@CFG,"+q[1]+","+q[2];
            if(q[1]=="I")reply+=",1,95,"+CarConfigClient.Hash(deviceValues);
            else if(q[1]=="G") {int index=int.Parse(q[3]);reply+=","+index;for(int i=index;i<Math.Min(95,index+8);i++)reply+=","+deviceValues[i];}
            else throw new Exception("Unexpected write in read-only UI test");
            f.Later(20,CarConfigClient.Frame(reply));
        };
        AttachLink(fake,s=>{});
        if(!millimeters.Checked||rows.Count!=103)throw new Exception("Terminal layout values incomplete");
        Snapshot();RefreshState();
        if(fake.Writes.Count!=0||verified||actual!=null||!movement.All(b=>b.Enabled))throw new Exception("Connection requires configuration or claims unread values verified");
        fake.Sleep(600);session.PrepareNextMove();fake.Sleep(300);session.Poll();session.PrepareNextMove();
        if(!fake.Writes.Any(w=>w.Contains("@PING,"))||fake.Writes.Any(w=>w.Contains("@CFG,")))throw new Exception("Idle handshake missing or synchronizes configuration");
        Button forward=movement.Single(b=>b.Text.Contains("前进"));forward.PerformClick();forward.PerformClick();
        if(jobs.Count!=1)throw new Exception("Double click queued multiple movements");
        Action job;jobs.TryDequeue(out job);job();
        if(fake.Writes.Count(w=>w.Contains("@MOVE,"))!=1||!outcome.Contains("已完成"))throw new Exception("UI did not execute one protected action");
        foreach(string mode in new[]{"R","F"}) {
            RefreshState();distance.Value=999;millimeters.Checked=true;rotationCounts.Value=500;dropRotationResult=mode=="F";
            Button rotate=movement.Single(b=>b.Text.Contains(mode));int before=fake.Writes.Count(w=>w.Contains("@MOVE,"));
            rotate.PerformClick();rotate.PerformClick();forward.PerformClick();
            if(jobs.Count!=1)throw new Exception("Rotation double click or mixed click queued more than one action");
            jobs.TryDequeue(out job);job();
            if(fake.Writes.Count(w=>w.Contains("@MOVE,"))!=before+1||!fake.Writes.Last(w=>w.Contains("@MOVE,")).TrimStart().StartsWith("@MOVE,"+mode+",355,CNT,")||!outcome.Contains("已完成")||!outcome.Contains("实际500.00")||!outcome.Contains("误差 0.00"))throw new Exception("Compensated rotation lost goal/wire/result semantics");
        }
        dropRotationResult=false;
        if(!fake.Writes.Any(w=>w.Contains("@RESULT,")))throw new Exception("Compensated result recovery not exercised");
        rotationCounts.Value=145;RefreshState();int writesBefore=fake.Writes.Count;
        SendMove("R");jobs.TryDequeue(out job);job();
        if(fake.Writes.Count!=writesBefore||!outcome.Contains("旋转目标必须大于提前量"))throw new Exception("Nonpositive compensated MOVE transmitted");
        rotationBrake.Enabled=false;FillRotation();RefreshState();SendMove("R");jobs.TryDequeue(out job);job();
        if(!fake.Writes.Last(w=>w.Contains("@MOVE,")).TrimStart().StartsWith("@MOVE,R,145,CNT,"))throw new Exception("Raw rotation did not preserve input");
        rotationBrake.Enabled=true;FillRotation();rotationCounts.Value=500;
        rotationUnit.SelectedIndex=0;
        if(rotationCounts.Value!=30)throw new Exception("Unit switch lost degree input");
        foreach(string mode in new[]{"R","F"}) {
            foreach(decimal angle in new[]{20m,mode=="R"?50m:47m,90m}) {
                rotationCounts.Value=angle;RefreshState();int before=fake.Writes.Count(w=>w.Contains("@MOVE,"));
                dropRotationResult=mode=="F";int expected=rotationBrake.WireCounts(mode,rotationBrake.GoalCounts(mode,angle));
                SendMove(mode);if(!jobs.TryDequeue(out job))throw new Exception("Degree move not queued");job();
                if(fake.Writes.Count(w=>w.Contains("@MOVE,"))!=before+1||!fake.Writes.Last(w=>w.Contains("@MOVE,")).TrimStart().StartsWith("@MOVE,"+mode+","+expected+",CNT,")||!outcome.Contains("°指令已完成"))throw new Exception("Degree MOVE conversion/result recovery failed");
            }
        }
        dropRotationResult=false;rotationCounts.Value=0.1m;writesBefore=fake.Writes.Count;
        SendMove("R");jobs.TryDequeue(out job);job();if(fake.Writes.Count!=writesBefore)throw new Exception("Tiny degree clamped into unintended motion");
        rotationCounts.Value=30;rotationBrake.Enabled=false;writesBefore=fake.Writes.Count;
        SendMove("R");jobs.TryDequeue(out job);job();if(fake.Writes.Count!=writesBefore||!outcome.Contains("开启旋转补偿"))throw new Exception("Disabled brake changed degree calibration semantics");
        rotationBrake.Enabled=true;FillRotation();rotationUnit.SelectedIndex=1;rotationCounts.Value=500;
        if(fake.Writes.Any(w=>w.Contains("@CFG,"))||verified)throw new Exception("Normal motion synchronized configuration");
        ReadDevice();Snapshot();RefreshState();
        if(!verified||!movement.All(b=>b.Enabled)||actual==null)throw new Exception("Manual readback failed");
        int original=config.Values[0];config.Values[0]=original+100;client.Invalidate();Snapshot();RefreshState();
        int beforeMismatch=fake.Writes.Count(w=>w.Contains("@CFG,"));
        forward.PerformClick();if(!jobs.TryDequeue(out job))throw new Exception("Local configuration mismatch blocked motion");job();
        if(!outcome.Contains("已完成")||verified||fake.Writes.Count(w=>w.Contains("@CFG,"))!=beforeMismatch)throw new Exception("Mismatched configuration forced sync");
        config.Values[0]=original;
        dropConfig=true;bool readFailed=false;try{ReadDevice();}catch(TimeoutException){readFailed=true;}
        Snapshot();RefreshState();if(!readFailed||verified||!movement.All(b=>b.Enabled))throw new Exception("Failed optional read locked movement");
        dropConfig=false;
        fake.Later(0,"MECANUM UNIVERSAL V6.3 COMM READY\r\n");session.Poll();Snapshot();RefreshState();
        if(actual!=null||verified||!movement.All(b=>b.Enabled))throw new Exception("Idle restart kept stale device values or required sync");
        dirty=true;RefreshState();forward.PerformClick();if(jobs.Count!=0||movement.Any(b=>b.Enabled))throw new Exception("Dirty editor permits motion");
        dirty=false;stopOnMove=true;
        foreach(Button button in new[]{forward,movement.Single(b=>b.Text.Contains(" R")),movement.Single(b=>b.Text.Contains(" F"))}) {
            RefreshState();int stops=fake.Writes.Count(w=>w=="!\r\nX\r\n");button.PerformClick();jobs.TryDequeue(out job);job();RefreshState();
            if(!uncertain||!confirmButton.Enabled||movement.Any(b=>b.Enabled)||fake.Writes.Count(w=>w=="!\r\nX\r\n")!=stops+1)throw new Exception("UI stop interlock failed");
            confirmButton.PerformClick();jobs.TryDequeue(out job);job();RefreshState();
            if(uncertain||!movement.All(b=>b.Enabled))throw new Exception("UI confirmation failed");
        }
        stopRequested=true;session.Poll();Snapshot();Disconnect();Snapshot();if(!unresolved||!uncertain)throw new Exception("Disconnect discarded uncertain movement");
        unresolved=false;uncertain=false;stopRequested=false;dirty=false;
        Console.WriteLine("PASS: UI compensated R/F goal500-wire355-final500, dropped result recovery without MOVE replay, nonpositive target rejection before TX, raw bypass, no-CFG motion, idle PING, manual readback/mismatch/restart availability, stop interlocks; fake transport only.");
    }
}

static class CarTerminalTests {
    public static void Run() {
        var fixture=new CarParameterFile{Values=(int[])CarParameterSchema.Defaults.Clone(),CountsPerMm=CarTerminalModel.Modes.ToDictionary(c=>c.ToString(),c=>7.3864m)};
        var parsed=CarTerminalModel.FromCells(i=>CarTerminalModel.Value(fixture,i));
        if(!parsed.Values.SequenceEqual(fixture.Values)||parsed.ToCounts("D",100)!=739)throw new Exception("Terminal conversion failed");
        parsed=CarTerminalModel.FromCells(i=>i==19?"600":i==98?"8.0":CarTerminalModel.Value(fixture,i));
        if(parsed.Values[19]!=600||parsed.ToCounts("D",100)!=800)throw new Exception("Terminal edited values failed");
        foreach(string invalid in new[]{"abc","-1","2001","500.5"}) {
            bool rejected=false;try{CarTerminalModel.FromCells(i=>i==19?invalid:CarTerminalModel.Value(fixture,i));}catch{rejected=true;}
            if(!rejected)throw new Exception("Terminal invalid value accepted");
        }
        string dir=Path.Combine(Path.GetTempPath(),"car_terminal_test_"+Guid.NewGuid().ToString("N"));Directory.CreateDirectory(dir);
        string path=Path.Combine(dir,"底盘参数.json");fixture.Save(path,fixture.Values);string original=File.ReadAllText(path,Encoding.UTF8);
        string saved=CarTerminalModel.Save(path,parsed,original);var reloaded=CarParameterFile.Load(path);
        if(reloaded.Values[19]!=600||reloaded.CountsPerMm["D"]!=8m||Directory.GetFiles(Path.Combine(dir,"backups")).Length!=1)throw new Exception("Terminal saved values failed");
        File.AppendAllText(path," ");bool conflict=false;try{CarTerminalModel.Save(path,fixture,saved);}catch{conflict=true;}
        if(!conflict)throw new Exception("External file edit was overwritten");
        var brake=new RotationBrakeSettings{R=140,F=150};
        if(brake.GoalCounts("R",20)!=500||brake.GoalCounts("R",50)!=1000||brake.GoalCounts("F",20)!=500||brake.GoalCounts("F",47)!=1000
            ||brake.GoalCounts("R",90)!=1667||brake.GoalCounts("F",90)!=1796||brake.GoalCounts("R",10)!=250||brake.GoalCounts("R",35)!=750)throw new Exception("Angle anchors/interpolation/extrapolation failed");
        int previous=0;for(int a=1;a<=360;a++){int goal=brake.GoalCounts("R",a);if(goal<=previous)throw new Exception("Angle map not monotone");previous=goal;}
        if(brake.WireCounts("R",500)!=360||brake.WireCounts("F",1000)!=850||brake.WireCounts("R",7000)!=6860)throw new Exception("Rotation brake direction/long-target conversion failed");
        foreach(int count in new[]{1,140}){bool rejected=false;try{brake.WireCounts("R",count);}catch{rejected=true;}if(!rejected)throw new Exception("Nonpositive compensation silently clamped");}
        brake.Enabled=false;brake.R500=21.5m;brake.F1000=48;if(brake.WireCounts("F",1)!=1)throw new Exception("Raw bypass failed");
        string brakePath=Path.Combine(dir,"旋转停车补偿.json");
        string brakeOriginal="{\"schema\":1,\"enabled\":true,\"r_lead_cnt\":145,\"f_lead_cnt\":145}";File.WriteAllText(brakePath,brakeOriginal);
        string brakeSaved=brake.Save(brakePath,brakeOriginal);
        if(!RotationBrakeSettings.Load(brakePath).Same(brake)||Directory.GetFiles(Path.Combine(dir,"backups")).Length!=2)throw new Exception("Rotation settings not persisted/backed up");
        File.AppendAllText(brakePath," ");conflict=false;try{brake.Save(brakePath,brakeSaved);}catch{conflict=true;}if(!conflict)throw new Exception("Rotation settings external edit overwritten");
        foreach(string bad in new[]{"145.5","-1","2001","\"145\""}) {
            File.WriteAllText(brakePath,brakeOriginal.Replace("\"r_lead_cnt\":145","\"r_lead_cnt\":"+bad));bool rejected=false;
            try{RotationBrakeSettings.Load(brakePath);}catch{rejected=true;}if(!rejected)throw new Exception("Invalid rotation settings accepted");
        }
        var badAngles=new RotationBrakeSettings{R500=50,R1000=20};bool invalidAngles=false;try{badAngles.Validate();}catch{invalidAngles=true;}if(!invalidAngles)throw new Exception("Nonmonotone calibration accepted");
        Console.WriteLine("PASS: angle anchors and monotone mapping, interpolation/extrapolation, schema1 compatibility/schema2 persisted angles, lead exactly once, bounds/save/backup/conflict.");
        using(var form=new CarTerminalForm(false)){form.Show();Application.DoEvents();form.OfflineChecks();form.Close();}
        Console.WriteLine("PASS: terminal edited values, precision/range checks, MM conversion, atomic save/backup, external modification conflict. No serial port opened.");
    }
}
