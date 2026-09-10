param([switch]$Test)
$ErrorActionPreference='Stop'
$destination=Join-Path (Split-Path $PSScriptRoot -Parent) '底盘集成终端.exe'
$sources=@('CarTerminal.cs','CarConfigClient.cs','CarParameterSchema.cs','CarConfigTests.cs','CarLinkProtocol.cs','CarLinkTests.cs') | ForEach-Object {Join-Path $PSScriptRoot $_}
& 'C:/Windows/Microsoft.NET/Framework64/v4.0.30319/csc.exe' /nologo /target:winexe /main:CarTerminal /optimize+ /reference:System.Web.Extensions.dll /reference:System.Windows.Forms.dll /reference:System.Drawing.dll "/out:$destination" $sources
if($LASTEXITCODE -ne 0){throw 'Terminal build failed'}
if($Test){
    $process=Start-Process -FilePath $destination -ArgumentList '--test' -WindowStyle Hidden -Wait -PassThru
    if($process.ExitCode -ne 0){throw 'Terminal offline tests failed'}
}
Write-Output 'Terminal build passed. No serial port opened.'
