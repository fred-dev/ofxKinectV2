# Builds libfreenect2 for ofxKinectV2 on Windows (x64, Release) with CUDA + OpenCL + OpenGL
# + CPU pipelines and TurboJPEG, then copies headers, import lib and runtime DLLs into
# libs/libfreenect2. The settings here are the reference for the macOS / Linux scripts:
# keep the CMake options in sync with build_libfreenect2_macos.sh and build_libfreenect2_linux.sh.
#
# Requirements
#   - libfreenect2 source  (git clone https://github.com/OpenKinect/libfreenect2)
#   - Visual Studio 2022 or newer with the C++ workload (vcpkg ships with it)
#   - CUDA Toolkit 12.x (also provides the OpenCL headers / OpenCL.lib)
#
# Usage (PowerShell):
#   .\build_libfreenect2_windows.ps1 -Source C:\path\to\libfreenect2
#   .\build_libfreenect2_windows.ps1 -Source C:\path\to\libfreenect2 -NoCuda    # OpenCL only

param(
    [Parameter(Mandatory = $true)][string]$Source,
    [string]$CudaRoot = $env:CUDA_PATH,
    [switch]$NoCuda,
    [switch]$NoOpenCL,
    # GPU architectures for the CUDA pipeline: Maxwell (GTX 9xx) to Ada (RTX 40xx), plus PTX
    # for anything newer to JIT.
    [string[]]$CudaArchs = @("52", "61", "75", "86", "89")
)

# Native tools write warnings to stderr; Windows PowerShell 5.1 would turn those into
# terminating errors under "Stop", so exit codes are checked explicitly instead.
$ErrorActionPreference = "Continue"
$Source = (Resolve-Path $Source).Path
$Addon = (Resolve-Path "$PSScriptRoot\..").Path

# --- tools ------------------------------------------------------------------------------
$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
# libfreenect2's CMake files predate CMake 4, so prefer VS 2022's CMake 3.x.
$vs2022 = & $vswhere -version "[17.0,18.0)" -property installationPath | Select-Object -First 1
$vsLatest = & $vswhere -latest -property installationPath
$cmake = "$vs2022\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
$generator = "Visual Studio 17 2022"
if (-not (Test-Path $cmake)) { throw "Visual Studio 2022 (CMake 3.x + 'Visual Studio 17 2022' generator) not found" }
$vcpkg = "$vsLatest\VC\vcpkg\vcpkg.exe"
if (-not (Test-Path $vcpkg)) { $vcpkg = "$vs2022\VC\vcpkg\vcpkg.exe" }

# --- dependencies (libusb, libjpeg-turbo, glfw3) via vcpkg ------------------------------
$deps = "$Source\depends\vcpkg_ofx"
New-Item -ItemType Directory -Force $deps | Out-Null
Copy-Item "$PSScriptRoot\vcpkg.json" "$deps\vcpkg.json" -Force
Push-Location $deps
& $vcpkg install --triplet x64-windows --x-install-root=installed
if ($LASTEXITCODE -ne 0) { Pop-Location; throw "vcpkg install failed" }
Pop-Location
$d = "$deps\installed\x64-windows"

# --- CUDA ---------------------------------------------------------------------------------
$cmakeArgs = @(
    "-S", $Source, "-B", "$Source\build_ofx",
    "-G", $generator, "-A", "x64",
    "-DCMAKE_INSTALL_PREFIX=$Source\build_ofx\install",
    "-DBUILD_SHARED_LIBS=ON",
    "-DBUILD_EXAMPLES=ON",
    "-DBUILD_OPENNI2_DRIVER=OFF",
    "-DENABLE_CXX11=ON",
    "-DENABLE_OPENGL=ON",
    "-DENABLE_VAAPI=OFF",
    "-DENABLE_TEGRAJPEG=OFF",
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

if ($NoCuda -or -not $CudaRoot) {
    $cmakeArgs += "-DENABLE_CUDA=OFF"
} else {
    # CUDA 11+ no longer installs the samples; the kernels need helper_math.h from them.
    $samples = "$Source\depends\cuda_samples"
    if (-not (Test-Path "$samples\common\inc\helper_math.h")) {
        New-Item -ItemType Directory -Force "$samples\common\inc" | Out-Null
        Invoke-WebRequest "https://raw.githubusercontent.com/NVIDIA/cuda-samples/v12.1/Common/helper_math.h" -OutFile "$samples\common\inc\helper_math.h"
    }
    $env:NVCUDASAMPLES_ROOT = $samples
    # CUDA < 12.4 rejects MSVC 19.40+ (nvcc host check, and the MSVC STL's STL1002 assert).
    # The kernels are plain C, so overriding both is safe; with CUDA 12.4+ these are no-ops.
    $nvcc = @("-allow-unsupported-compiler", "-D_ALLOW_COMPILER_AND_STL_VERSION_MISMATCH")
    foreach ($a in $CudaArchs) { $nvcc += "-gencode=arch=compute_$a,code=sm_$a" }
    $last = $CudaArchs[-1]
    $nvcc += "-gencode=arch=compute_$last,code=compute_$last"
    $cmakeArgs += @("-DENABLE_CUDA=ON", "-DCUDA_TOOLKIT_ROOT_DIR=$CudaRoot", "-DCUDA_NVCC_FLAGS=$($nvcc -join ';')")
}

if ($NoOpenCL -or -not $CudaRoot) {
    if (-not $NoOpenCL) { Write-Warning "No CUDA_PATH: OpenCL headers not found, building without OpenCL" }
    $cmakeArgs += "-DENABLE_OPENCL=OFF"
} else {
    $cmakeArgs += @("-DENABLE_OPENCL=ON", "-DOpenCL_INCLUDE_DIR=$CudaRoot\include", "-DOpenCL_LIBRARY=$CudaRoot\lib\x64\OpenCL.lib")
}

# --- configure, build, install ------------------------------------------------------------
# CMake reads backslashes in cache values as escapes ("\P" in "C:\Program Files" breaks FindCUDA).
$cmakeArgs = $cmakeArgs | ForEach-Object { if ($_ -like "-D*") { $_ -replace '\\', '/' } else { $_ } }
Remove-Item "$Source\build_ofx\CMakeCache.txt" -ErrorAction SilentlyContinue
& $cmake @cmakeArgs
if ($LASTEXITCODE -ne 0) { throw "CMake configure failed" }
# Release and Debug: libfreenect2's C++ API passes std::string across the DLL boundary, and a
# Debug app's std::string doesn't match a Release DLL's, so Debug builds need their own
# library (freenect2d). Its C dependencies (libusb, turbojpeg, glfw) are shared.
foreach ($config in @("Release", "Debug")) {
    & $cmake --build "$Source\build_ofx" --config $config --parallel
    if ($LASTEXITCODE -ne 0) { throw "$config build failed" }
    & $cmake --install "$Source\build_ofx" --config $config
    if ($LASTEXITCODE -ne 0) { throw "$config install failed" }
}

# --- copy into the addon ------------------------------------------------------------------
# The project generator links lib/vs/x64/Release/*.lib into Release configurations and
# lib/vs/x64/Debug/*.lib into Debug ones, and copies every DLL under lib/vs/x64 next to the exe.
$inst = "$Source\build_ofx\install"
$incOut = "$Addon\libs\libfreenect2\include\vs\libfreenect2"
$libOut = "$Addon\libs\libfreenect2\lib\vs\x64"
Remove-Item -Recurse -Force $incOut, $libOut -ErrorAction SilentlyContinue
New-Item -ItemType Directory -Force $incOut, "$libOut\Release", "$libOut\Debug" | Out-Null
Copy-Item "$inst\include\libfreenect2\*" $incOut -Recurse
Copy-Item "$inst\lib\freenect2.lib" "$libOut\Release"
Copy-Item "$inst\lib\freenect2d.lib" "$libOut\Debug"
# Runtime: libfreenect2 + its dependencies.
Copy-Item "$inst\bin\freenect2.dll" $libOut
Copy-Item "$inst\bin\freenect2d.dll" $libOut
# turbojpeg 3 links spng (for its PNG load/save API), which links zlib.
foreach ($dll in @("libusb-1.0.dll", "turbojpeg.dll", "spng.dll", "z.dll", "glfw3.dll")) { Copy-Item "$d\bin\$dll" $libOut }

Write-Host ""
Write-Host "libfreenect2 installed into $Addon\libs\libfreenect2 (pipelines: CPU OpenGL$(if (-not $NoOpenCL -and $CudaRoot) {' OpenCL'})$(if (-not $NoCuda -and $CudaRoot) {' CUDA'}))"
