# Builds libfreenect2-metal for ofxKinectV2 on Windows (x64, Release + Debug) with the CUDA,
# OpenCL, OpenGL and CPU depth pipelines and TurboJPEG, then copies headers, import libs and
# runtime DLLs into libs/libfreenect2. The settings here are the reference for the macOS /
# Linux scripts: keep the CMake options in sync with build_libfreenect2_macos.sh and
# build_libfreenect2_linux.sh.
#
# Requirements
#   - libfreenect2-metal source (git clone https://github.com/hbmartin/libfreenect2-metal) -
#     a maintained fork of OpenKinect/libfreenect2 with Metal (Apple Silicon), the 0.4 API,
#     recording / replay and self-contained CUDA. Plain upstream libfreenect2 is not supported
#     any more by this script.
#   - Visual Studio 2022 or newer with the C++ workload. The newest one installed is used (its
#     compiler should match the one openFrameworks is built with), with its own CMake, Ninja
#     and vcpkg.
#   - CUDA Toolkit 12.x (also provides the OpenCL headers / OpenCL.lib)
#
# Usage (PowerShell):
#   .\build_libfreenect2_windows.ps1 -Source C:\path\to\libfreenect2-metal
#   .\build_libfreenect2_windows.ps1 -Source C:\path\to\libfreenect2-metal -NoCuda    # OpenCL only

param(
    [Parameter(Mandatory = $true)][string]$Source,
    [string]$CudaRoot = $env:CUDA_PATH,
    [switch]$NoCuda,
    [switch]$NoOpenCL,
    # GPU architectures for the CUDA pipeline: Maxwell (GTX 9xx) to Ada (RTX 40xx); the last one
    # also gets PTX so newer GPUs can JIT it.
    [string[]]$CudaArchs = @("52", "61", "75", "86", "89")
)

# Native tools write warnings to stderr; Windows PowerShell 5.1 would turn those into
# terminating errors under "Stop", so exit codes are checked explicitly instead.
$ErrorActionPreference = "Continue"
$Source = (Resolve-Path $Source).Path
$Addon = (Resolve-Path "$PSScriptRoot\..").Path
$build = "$Source\build_ofx"

# --- Visual Studio developer environment ---------------------------------------------------
# Ninja instead of a Visual Studio generator: CMake's CUDA language support under a VS
# generator needs the CUDA MSBuild integration for that exact VS version, which older CUDA
# toolkits don't have for new VS releases. Ninja calls nvcc directly.
$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
$vsLatest = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $vsLatest) { throw "Visual Studio with the C++ workload not found" }
$vs = $vsLatest
# nvcc's front end has to parse the MSVC standard library headers. CUDA before 12.8 can't
# parse Visual Studio 2026's (MSVC 14.5x), so with an older toolkit build with Visual Studio
# 2022's compiler instead. MSVC is binary compatible from 2015 to 2026: the DLL works in
# apps built with the newer compiler.
if (-not $NoCuda -and $CudaRoot -and (Test-Path "$CudaRoot\version.json")) {
    $cudaVersion = [version]((Get-Content "$CudaRoot\version.json" -Raw | ConvertFrom-Json).cuda.version)
    $vs2022 = & $vswhere -version "[17.0,18.0)" -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath | Select-Object -First 1
    if ($cudaVersion -lt [version]"12.8" -and $vs2022) {
        Write-Host "CUDA $($cudaVersion) - building with Visual Studio 2022's compiler"
        $vs = $vs2022
    }
}
Import-Module "$vs\Common7\Tools\Microsoft.VisualStudio.DevShell.dll"
Enter-VsDevShell -VsInstallPath $vs -SkipAutomaticLocation -DevCmdArguments "-arch=x64 -host_arch=x64" | Out-Null
$cmake = "$vs\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
$vcpkg = "$vsLatest\VC\vcpkg\vcpkg.exe"
Write-Host "Visual Studio: $vs"

# --- dependencies (libusb, libjpeg-turbo, glfw3) via vcpkg ------------------------------
$deps = "$Source\depends\vcpkg_ofx"
New-Item -ItemType Directory -Force $deps | Out-Null
Copy-Item "$PSScriptRoot\vcpkg.json" "$deps\vcpkg.json" -Force
Push-Location $deps
& $vcpkg install --triplet x64-windows --x-install-root=installed
if ($LASTEXITCODE -ne 0) { Pop-Location; throw "vcpkg install failed" }
Pop-Location
$d = "$deps\installed\x64-windows"

$cmakeArgs = @(
    "-S", $Source, "-B", $build,
    "-G", "Ninja Multi-Config",
    "-DCMAKE_INSTALL_PREFIX=$build\install",
    "-DBUILD_SHARED_LIBS=ON",
    "-DBUILD_EXAMPLES=ON",
    "-DBUILD_OPENNI2_DRIVER=OFF",
    "-DBUILD_TESTING=OFF",
    "-DBUILD_CALIBRATION_TOOLS=OFF",
    "-DENABLE_OPENGL=ON",
    "-DENABLE_METAL=OFF",
    "-DENABLE_VAAPI=OFF",
    "-DENABLE_TEGRAJPEG=OFF",
    # libfreenect2-metal's CI doesn't build on Windows: windows.h's min / max macros break its
    # std::max / numeric_limits<>::max() calls. _INIT keeps CMake's default MSVC flags.
    "-DCMAKE_CXX_FLAGS_INIT=/DNOMINMAX",
    "-DCMAKE_C_FLAGS_INIT=/DNOMINMAX",
    "-DLibUSB_INCLUDE_DIRS=$d\include\libusb-1.0",
    "-DLibUSB_LIBRARIES=$d\lib\libusb-1.0.lib",
    "-DLibUSB_DLL=$d\bin\libusb-1.0.dll",
    "-DTurboJPEG_INCLUDE_DIRS=$d\include",
    "-DTurboJPEG_LIBRARIES=$d\lib\turbojpeg.lib",
    "-DTurboJPEG_DLL=$d\bin\turbojpeg.dll",
    "-DGLFW3_INCLUDE_DIRS=$d\include",
    "-DGLFW3_LIBRARIES=$d\lib\glfw3dll.lib",
    "-DGLFW3_DLL=$d\bin\glfw3.dll"
)

# --- CUDA (native CMake CUDA language; no CUDA samples needed) -----------------------------
if ($NoCuda -or -not $CudaRoot) {
    $cmakeArgs += "-DENABLE_CUDA=OFF"
} else {
    $archs = ($CudaArchs | Select-Object -SkipLast 1 | ForEach-Object { "$_-real" }) + @($CudaArchs[-1])
    # libfreenect2-metal's CUDA kernels use the POSIX uint / ushort typedefs (from
    # <sys/types.h> on Linux; upstream got them from helper_math.h, which the fork dropped).
    # Windows has neither, so force-include them into the CUDA compile.
    New-Item -ItemType Directory -Force $build | Out-Null
    $compat = "$build\ofx_cuda_windows_compat.h"
    Set-Content -Encoding ascii $compat "#pragma once`n// generated by ofxKinectV2/scripts/build_libfreenect2_windows.ps1`ntypedef unsigned int uint;`ntypedef unsigned short ushort;`n"
    # CUDA < 12.4 rejects MSVC 19.40+ (nvcc's host compiler check, and the MSVC STL's STL1002
    # assert). The kernels are plain C, so overriding both is safe; with CUDA 12.4+ they're no-ops.
    $cmakeArgs += @(
        "-DENABLE_CUDA=ON",
        "-DCMAKE_CUDA_COMPILER=$CudaRoot\bin\nvcc.exe",
        "-DCUDAToolkit_ROOT=$CudaRoot",
        "-DCMAKE_CUDA_ARCHITECTURES=$($archs -join ';')",
        "-DCMAKE_CUDA_FLAGS=-allow-unsupported-compiler -D_ALLOW_COMPILER_AND_STL_VERSION_MISMATCH -DNOMINMAX -include $compat"
    )
}

if ($NoOpenCL -or -not $CudaRoot) {
    if (-not $NoOpenCL) { Write-Warning "No CUDA_PATH: OpenCL headers not found, building without OpenCL" }
    $cmakeArgs += "-DENABLE_OPENCL=OFF"
} else {
    $cmakeArgs += @("-DENABLE_OPENCL=ON", "-DOpenCL_INCLUDE_DIR=$CudaRoot\include", "-DOpenCL_LIBRARY=$CudaRoot\lib\x64\OpenCL.lib")
}

# --- configure, build, install ------------------------------------------------------------
# CMake reads backslashes in cache values as escapes ("\P" in "C:\Program Files").
$cmakeArgs = $cmakeArgs | ForEach-Object { if ($_ -like "-D*") { $_ -replace '\\', '/' } else { $_ } }
if (Test-Path "$build\CMakeCache.txt") { Remove-Item -LiteralPath "$build\CMakeCache.txt" }
if (Test-Path "$build\CMakeFiles") { Remove-Item -LiteralPath "$build\CMakeFiles" -Recurse -Force }
& $cmake @cmakeArgs
if ($LASTEXITCODE -ne 0) { throw "CMake configure failed" }
# Release and Debug: libfreenect2's C++ API passes std::string across the DLL boundary, and a
# Debug app's std::string doesn't match a Release DLL's, so Debug builds need their own
# library (freenect2d). Its C dependencies (libusb, turbojpeg, glfw) are shared.
foreach ($config in @("Release", "Debug")) {
    & $cmake --build $build --config $config --parallel
    if ($LASTEXITCODE -ne 0) { throw "$config build failed" }
    & $cmake --install $build --config $config
    if ($LASTEXITCODE -ne 0) { throw "$config install failed" }
}

# --- copy into the addon ------------------------------------------------------------------
# The project generator links lib/vs/x64/Release/*.lib into Release configurations and
# lib/vs/x64/Debug/*.lib into Debug ones, and copies every DLL under lib/vs/x64 next to the exe.
$inst = "$build\install"
$incOut = "$Addon\libs\libfreenect2\include\vs\libfreenect2"
$libOut = "$Addon\libs\libfreenect2\lib\vs\x64"
foreach ($dir in @($incOut, $libOut)) { if (Test-Path $dir) { Remove-Item -LiteralPath $dir -Recurse -Force } }
New-Item -ItemType Directory -Force $incOut, "$libOut\Release", "$libOut\Debug" | Out-Null
Copy-Item "$inst\include\libfreenect2\*" $incOut -Recurse
Copy-Item "$inst\lib\freenect2.lib" "$libOut\Release"
Copy-Item "$inst\lib\freenect2d.lib" "$libOut\Debug"
# Runtime: libfreenect2 + its dependencies.
Copy-Item "$inst\bin\freenect2.dll" $libOut
Copy-Item "$inst\bin\freenect2d.dll" $libOut
# turbojpeg 3 links spng (for its PNG load/save API), which links zlib.
foreach ($dll in @("libusb-1.0.dll", "turbojpeg.dll", "spng.dll", "z.dll", "glfw3.dll")) { Copy-Item "$d\bin\$dll" $libOut }
# libfreenect2-metal links the shared CUDA runtime: ship it (NVIDIA allows redistributing it),
# otherwise freenect2.dll won't even load on a machine without the CUDA toolkit. Without an
# NVIDIA GPU the CUDA pipeline just reports itself unusable and OpenCL / CPU are used.
if (-not $NoCuda -and $CudaRoot) { Copy-Item "$CudaRoot\bin\cudart64_*.dll" $libOut }

$revision = (git -C $Source rev-parse --short HEAD) 2>$null
Write-Host ""
Write-Host "libfreenect2-metal $revision installed into $Addon\libs\libfreenect2 (pipelines: CPU OpenGL$(if (-not $NoOpenCL -and $CudaRoot) {' OpenCL'})$(if (-not $NoCuda -and $CudaRoot) {' CUDA'}))"
