#define AppName "Synapse Qualisys Bridge"
#ifndef Version
#define Version "dev"
#endif
#ifndef SourceDir
#define SourceDir "."
#endif
#ifndef OutputDir
#define OutputDir "."
#endif

[Setup]
AppId={{C63E3D1E-3E05-4374-9BA1-8C43FA19F90F}
AppName={#AppName}
AppVersion={#Version}
AppPublisher=CogniPilot
AppPublisherURL=https://github.com/CogniPilot/synapse_qualisys_bridge
AppSupportURL=https://github.com/CogniPilot/synapse_qualisys_bridge/issues
AppUpdatesURL=https://github.com/CogniPilot/synapse_qualisys_bridge/releases
DefaultDirName={localappdata}\Programs\Synapse Qualisys Bridge
DefaultGroupName=Synapse Qualisys Bridge
DisableProgramGroupPage=yes
OutputDir={#OutputDir}
OutputBaseFilename=synapse-qualisys-bridge-{#Version}-windows-x86_64-setup
Compression=lzma2
SolidCompression=yes
WizardStyle=modern
PrivilegesRequired=lowest
ArchitecturesAllowed=x64
ArchitecturesInstallIn64BitMode=x64
UninstallDisplayIcon={app}\synapse-qualisys-bridge.exe
CloseApplications=yes
RestartApplications=yes

[Tasks]
Name: "startup"; Description: "Start Synapse Qualisys Bridge when I sign in"; Flags: unchecked

[Files]
Source: "{#SourceDir}\synapse-qualisys-bridge.exe"; DestDir: "{app}"; Flags: ignoreversion
Source: "{#SourceDir}\README.md"; DestDir: "{app}"; Flags: ignoreversion
Source: "{#SourceDir}\LICENSE"; DestDir: "{app}"; Flags: ignoreversion
Source: "{#SourceDir}\SYNAPSE_FBS_COMPATIBILITY.txt"; DestDir: "{app}"; Flags: ignoreversion
Source: "{#SourceDir}\bridge.toml"; DestDir: "{localappdata}\Synapse Qualisys Bridge"; Flags: onlyifdoesntexist

[Dirs]
Name: "{localappdata}\Synapse Qualisys Bridge"
Name: "{localappdata}\Synapse Qualisys Bridge\logs"

[Icons]
Name: "{autoprograms}\Synapse Qualisys Bridge\Synapse Qualisys Bridge"; Filename: "{app}\synapse-qualisys-bridge.exe"; Parameters: "--config ""{localappdata}\Synapse Qualisys Bridge\bridge.toml"" --open"; WorkingDir: "{app}"
Name: "{autoprograms}\Synapse Qualisys Bridge\Open Dashboard"; Filename: "http://127.0.0.1:8787"
Name: "{autoprograms}\Synapse Qualisys Bridge\View Logs"; Filename: "{localappdata}\Synapse Qualisys Bridge\logs"
Name: "{autoprograms}\Synapse Qualisys Bridge\Edit Config"; Filename: "{localappdata}\Synapse Qualisys Bridge\bridge.toml"
Name: "{autoprograms}\Synapse Qualisys Bridge\Uninstall"; Filename: "{uninstallexe}"
Name: "{userstartup}\Synapse Qualisys Bridge"; Filename: "{app}\synapse-qualisys-bridge.exe"; Parameters: "--config ""{localappdata}\Synapse Qualisys Bridge\bridge.toml"""; WorkingDir: "{app}"; Tasks: startup

[Run]
Filename: "{app}\synapse-qualisys-bridge.exe"; Parameters: "--config ""{localappdata}\Synapse Qualisys Bridge\bridge.toml"" --open"; Description: "Launch Synapse Qualisys Bridge"; Flags: nowait postinstall skipifsilent
