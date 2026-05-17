param(
    [string]$Distro = "Ubuntu-22.04",
    [string]$WslRepo = "/home/branc/aerion_ros2_ws/src/airsim_ros2_bridge",
    [string]$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
)

$ErrorActionPreference = "Stop"

function Convert-ToWslUncPath {
    param(
        [string]$DistroName,
        [string]$LinuxPath
    )

    $relativePath = $LinuxPath.TrimStart("/") -replace "/", "\"
    return "\\wsl$\" + $DistroName + "\" + $relativePath
}

$destinationRoot = Convert-ToWslUncPath -DistroName $Distro -LinuxPath $WslRepo

$files = @(
    @{
        Source = "airsim_ros2_bridge/airsim_ros2_bridge/bridge_node.py"
        Destination = "airsim_ros2_bridge/bridge_node.py"
    },
    @{
        Source = "airsim_ros2_bridge/airsim_ros2_bridge/drone_controller.py"
        Destination = "airsim_ros2_bridge/drone_controller.py"
    },
    @{
        Source = "docs/mavros_topic_mapping.md"
        Destination = "docs/mavros_topic_mapping.md"
    },
    @{
        Source = "scripts/generate_settings.py"
        Destination = "scripts/generate_settings.py"
    },
    @{
        Source = "scripts/smoke_airsim_ros2_bridge.sh"
        Destination = "scripts/smoke_airsim_ros2_bridge.sh"
    },
    @{
        Source = "scripts/run_airsim_ros2_bridge.sh"
        Destination = "scripts/run_airsim_ros2_bridge.sh"
    },
    @{
        Source = "scripts/run_airsim_ros2_bridge_instances.sh"
        Destination = "scripts/run_airsim_ros2_bridge_instances.sh"
    },
    @{
        Source = "scripts/launch_mavros_px4_instances.sh"
        Destination = "scripts/launch_mavros_px4_instances.sh"
    },
    @{
        Source = "scripts/launch_px4_instances.sh"
        Destination = "scripts/launch_px4_instances.sh"
    }
)

foreach ($file in $files) {
    $source = Join-Path $RepoRoot ($file.Source -replace "/", [IO.Path]::DirectorySeparatorChar)
    $destination = Join-Path $destinationRoot ($file.Destination -replace "/", "\")
    $destinationDir = Split-Path -Parent $destination

    if (-not (Test-Path -LiteralPath $source)) {
        throw "Missing source file: $source"
    }

    if (-not (Test-Path -LiteralPath $destinationDir)) {
        New-Item -ItemType Directory -Path $destinationDir -Force | Out-Null
    }

    Copy-Item -LiteralPath $source -Destination $destination -Force
    Write-Host "Copied $($file.Source) -> $($file.Destination)"
}

wsl -d $Distro -- chmod +x "$WslRepo/scripts/smoke_airsim_ros2_bridge.sh" "$WslRepo/scripts/run_airsim_ros2_bridge.sh" "$WslRepo/scripts/run_airsim_ros2_bridge_instances.sh" "$WslRepo/scripts/launch_mavros_px4_instances.sh" "$WslRepo/scripts/launch_px4_instances.sh"
Write-Host "Updated executable bits in $Distro"
