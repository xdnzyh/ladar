$ErrorActionPreference='Stop'
$destination=Join-Path (Split-Path $PSScriptRoot -Parent) '底盘参数工具_CONFIG1.exe'
$sources=@('CarConfigConsole.cs','CarConfigClient.cs','CarParameterSchema.cs','CarConfigTests.cs','CarLinkProtocol.cs','CarLinkTests.cs') | ForEach-Object {Join-Path $PSScriptRoot $_}
& 'C:/Windows/Microsoft.NET/Framework64/v4.0.30319/csc.exe' /nologo /target:exe /optimize+ /reference:System.Web.Extensions.dll "/out:$destination" $sources
if($LASTEXITCODE -ne 0){throw 'Build failed'}
Write-Output 'Build passed. No serial port opened.'
