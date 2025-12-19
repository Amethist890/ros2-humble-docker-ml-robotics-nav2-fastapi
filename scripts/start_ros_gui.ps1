# ROS 2 Humble GUI Launcher for Windows
# This script starts VcXsrv and launches the ROS container with GUI support

param(
    [switch]$StartXServer,
    [switch]$GPU,
    [string]$ContainerName = "ros_humble_dev"
)

$ErrorActionPreference = "Stop"

Write-Host "=== ROS 2 Humble GUI Launcher ===" -ForegroundColor Cyan

# Check if VcXsrv is installed
$vcxsrvPath = "C:\Program Files\VcXsrv\vcxsrv.exe"
if (-not (Test-Path $vcxsrvPath)) {
    Write-Host "VcXsrv not found at $vcxsrvPath" -ForegroundColor Yellow
    Write-Host "Please install VcXsrv from: https://sourceforge.net/projects/vcxsrv/" -ForegroundColor Yellow
}

# Start X Server if requested
if ($StartXServer) {
    Write-Host "Starting VcXsrv X Server..." -ForegroundColor Green
    
    # Kill existing VcXsrv processes
    Get-Process -Name "vcxsrv" -ErrorAction SilentlyContinue | Stop-Process -Force
    
    # Start VcXsrv with proper settings
    Start-Process -FilePath $vcxsrvPath -ArgumentList ":0", "-multiwindow", "-clipboard", "-wgl", "-ac"
    Start-Sleep -Seconds 2
    
    Write-Host "VcXsrv started successfully" -ForegroundColor Green
}

# Set DISPLAY environment variable
$env:DISPLAY = "host.docker.internal:0.0"
Write-Host "DISPLAY set to: $env:DISPLAY" -ForegroundColor Cyan

# Check if container exists
$existingContainer = docker ps -a --filter "name=$ContainerName" --format "{{.Names}}" 2>$null
if ($existingContainer -eq $ContainerName) {
    Write-Host "Container '$ContainerName' exists. Starting..." -ForegroundColor Yellow
    docker start -ai $ContainerName
}
else {
    Write-Host "Creating new container '$ContainerName'..." -ForegroundColor Green
    
    $dockerArgs = @(
        "run", "-it",
        "--name", $ContainerName,
        "--network", "host",
        "-v", "${PWD}/src:/home/dev/ros_ws/src",
        "-e", "DISPLAY=host.docker.internal:0.0",
        "-e", "QT_X11_NO_MITSHM=1"
    )
    
    if ($GPU) {
        Write-Host "Enabling GPU support..." -ForegroundColor Cyan
        $dockerArgs += "--gpus", "all"
        $dockerArgs += "ros_humble_gpu"
    }
    else {
        $dockerArgs += "osrf/ros:humble-desktop-full"
    }
    
    & docker $dockerArgs
}

Write-Host "Container session ended." -ForegroundColor Cyan

