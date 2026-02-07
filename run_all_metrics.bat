@echo off
REM Run all navigation algorithm metrics validation

echo ==========================================
echo Starting Full Metrics Validation Suite
echo ==========================================

REM VFH Algorithm
echo.
echo ^>^>^> Now running: VFH with 2 obstacles...
python metrics_validation.py --obstacles 2 --runs 1000 --navigation vfh
echo ^<^<^< Ended: VFH with 2 obstacles

echo.
echo ^>^>^> Now running: VFH with 5 obstacles...
python metrics_validation.py --obstacles 5 --runs 1000 --navigation vfh
echo ^<^<^< Ended: VFH with 5 obstacles

echo.
echo ^>^>^> Now running: VFH with 10 obstacles...
python metrics_validation.py --obstacles 10 --runs 1000 --navigation vfh
echo ^<^<^< Ended: VFH with 10 obstacles

echo.
echo ^>^>^> Now running: VFH with 20 obstacles...
python metrics_validation.py --obstacles 20 --runs 1000 --navigation vfh
echo ^<^<^< Ended: VFH with 20 obstacles

REM DWA Algorithm
echo.
echo ^>^>^> Now running: DWA with 2 obstacles...
python metrics_validation.py --obstacles 2 --runs 1000 --navigation dwa
echo ^<^<^< Ended: DWA with 2 obstacles

echo.
echo ^>^>^> Now running: DWA with 5 obstacles...
python metrics_validation.py --obstacles 5 --runs 1000 --navigation dwa
echo ^<^<^< Ended: DWA with 5 obstacles

echo.
echo ^>^>^> Now running: DWA with 10 obstacles...
python metrics_validation.py --obstacles 10 --runs 1000 --navigation dwa
echo ^<^<^< Ended: DWA with 10 obstacles

echo.
echo ^>^>^> Now running: DWA with 20 obstacles...
python metrics_validation.py --obstacles 20 --runs 1000 --navigation dwa
echo ^<^<^< Ended: DWA with 20 obstacles

REM GapNav Algorithm
echo.
echo ^>^>^> Now running: GapNav with 2 obstacles...
python metrics_validation.py --obstacles 2 --runs 1000 --navigation gapnav
echo ^<^<^< Ended: GapNav with 2 obstacles

echo.
echo ^>^>^> Now running: GapNav with 5 obstacles...
python metrics_validation.py --obstacles 5 --runs 1000 --navigation gapnav
echo ^<^<^< Ended: GapNav with 5 obstacles

echo.
echo ^>^>^> Now running: GapNav with 10 obstacles...
python metrics_validation.py --obstacles 10 --runs 1000 --navigation gapnav
echo ^<^<^< Ended: GapNav with 10 obstacles

echo.
echo ^>^>^> Now running: GapNav with 20 obstacles...
python metrics_validation.py --obstacles 20 --runs 1000 --navigation gapnav
echo ^<^<^< Ended: GapNav with 20 obstacles

REM Compare Metrics
echo.
echo ^>^>^> Now running: compare_metrics.py...
python compare_metrics.py
echo ^<^<^< Ended: compare_metrics.py

echo.
echo ==========================================
echo All metrics validation completed!
echo ==========================================
pause
