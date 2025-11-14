param (
    [Parameter(Mandatory = $true)]
    [string]$Name
)

function do_render {
    # Define input and output files
    param(
        [Parameter(Mandatory = $true)]
        [string]$BoardLayer,

        [Parameter(Mandatory = $true)]
        [string]$InputLayer
    )

    $FullPath = Resolve-Path $InputLayer
    $Directory = Split-Path -Parent $FullPath
    $BaseName = [System.IO.Path]::GetFileNameWithoutExtension($FullPath)
    $InputFile = Join-Path -Path $Directory -ChildPath "temp.png"
    $OutputFile = Join-Path -Path $Directory -ChildPath "$($BaseName).png"

    Write-Host "----- ${InputLayer} -----"

    if (Test-Path $InputFile) {
        Remove-Item $InputFile
    }

    $gerbvPath = "c:\bin\Gerb2.7\App\gerbv64\bin\gerbv.exe"
    $magickPath = "C:\bin\ImageMagick-7.1.1-Q16\magick.exe"

    # 4000 DPI
    # The board outline trace is 4mil so it will be 16 pixels wide
    # We'll crop to halfway through it (8 pixels off each side)

    # Board Layer in RED
    # Actual Layer in GREEN

    @( "`n" | & $gerbvPath -D 4000 -B 0 -a -b "#000000" -f "#00ff00ff" -f "#ff0000ff" -o "$InputFile" -x png "$InputLayer" "$BoardLayer" 2>NUL >NUL )

    $GerbvProcessName = [System.IO.Path]::GetFileNameWithoutExtension($gerbvPath)
    Write-Host "Waiting for Gerbv..."

    do {
        # Find ALL processes named 'gerbv' (parent and children)
        $RunningProcesses = Get-CimInstance -ClassName Win32_Process | 
        Where-Object { $_.Name -eq "$($GerbvProcessName).exe" } 

        if ($RunningProcesses) {
            Start-Sleep -Seconds 0.5
        }

    } while ($RunningProcesses)

    Write-Host "GerbV complete"

    # --- STEP 1: Find the extents of the Red pixels (Two-Pass Logic) ---

    # This command isolates the Red channel, negates it (so red areas are dark "content"),
    # trims the surrounding "background" (white/non-red area), and outputs the geometry (WxH+X+Y).
    # The -channel is not needed here; just use the R channel isolation.
    # A simpler and more direct approach is to get the bounding box of non-white pixels after isolating R.
    $RedExtent = & $magickPath $InputFile `
        -channel R -separate -delete "1,2" `
        -negate -trim -format "%@" info:

    # Check if any red pixels were found
    if (-not $RedExtent) {
        Write-Host "No red pixels found to define crop extent. Exiting."
        exit 1
    }

    # --- STEP 2: Parse the Bounding Box and Calculate New Geometry ---

    # Example $RedExtent: "100x200+10+20"
    $Parts = $RedExtent -match '(\d+)x(\d+)\+(\d+)\+(\d+)'
    if (-not $Parts) {
        Write-Host "Failed to parse extent geometry: $RedExtent"
        exit 1
    }

    # $matches[1] = Width, $matches[2] = Height, $matches[3] = X-Offset, $matches[4] = Y-Offset
    $Width = [int]$Matches[1]
    $Height = [int]$Matches[2]
    $XOffset = [int]$Matches[3]
    $YOffset = [int]$Matches[4]

    # Calculate the new geometry (2 pixels smaller means -1 on each edge, +1 offset)
    $NewWidth = $Width - 16
    $NewHeight = $Height - 16
    $NewX = $XOffset + 8
    $NewY = $YOffset + 8

    # Ensure dimensions are not negative (just in case the extent was 0x0 or 1x1)
    if ($NewWidth -lt 1 -or $NewHeight -lt 1) {
        Write-Host "Calculated crop size is too small after shaving 2 pixels. Exiting."
        exit 1
    }

    # Construct the final geometry string
    $CropGeometry = "${NewWidth}x${NewHeight}+${NewX}+${NewY}"

    # --- STEP 3: Apply All Transformations and Save ---

    $magick_args = @(
        $InputFile
        '-crop'
        $CropGeometry
        '+repage'
        '-channel'
        'R'
        '-evaluate'
        'set'
        '0'
        '+channel'
        '-channel'
        'G'
        '-separate'
        '-set'
        'colorspace'
        'Gray'
        '-combine'
        '-resize'
        '50%'
        $OutputFile
    )

    & $magickPath @magick_args
}

do_render "${name}_Board.gbr" "${name}_Copper_Signal_Top.gbr"
do_render "${name}_Board.gbr" "${name}_Copper_Signal_Bot.gbr"
do_render "${name}_Board.gbr" "${name}_Legend_Top.gbr"
do_render "${name}_Board.gbr" "${name}_Legend_Bot.gbr"
do_render "${name}_Board.gbr" "${name}_Paste_Top.gbr"
do_render "${name}_Board.gbr" "${name}_Paste_Bot.gbr"
do_render "${name}_Board.gbr" "${name}_Soldermask_Top.gbr"
do_render "${name}_Board.gbr" "${name}_Soldermask_Bot.gbr"
do_render "${name}_Board.gbr" "${name}_PTH_Drill.gbr"
do_render "${name}_Board.gbr" "${name}_NPTH_Drill.gbr"
