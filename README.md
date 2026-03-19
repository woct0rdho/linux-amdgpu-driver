# Fork of amdgpu mainline driver for PC sampling on gfx1151 (Strix Halo)

Tested with Ubuntu 26.04, Linux kernel 7.0 , nightly ROCm from TheRock as of 20260303. Some unit tests are not passed yet. Use at your own risk.

`dkms` branch contains the conversion from source tree to dkms, and fixes for a double free issue when failed to create pid kobj.

`pc_sampling_gfx1151` branch is based on `dkms` branch, and contains the actual implementation of PC sampling on gfx1151. Previously the mainline driver did not contain support for PC sampling, so I ported it from the non-mainline driver (old repo https://github.com/woct0rdho/amdgpu ).

`pc_sampling_gfx1151_nocwsr` branch contains an implementation of PC sampling on gfx1151 before the CWSR issue was fixed, where we make the trap handler a no-op and directly read information about the waves from SQ_IND. When the waves stall, the results can be different from the current approach.

Both host-trap sampling and stochastic sampling seem to work.

Bonus: Thread tracing also seems to work. Double buffer in thread tracing is not implemented yet.

## Installation

1. Clone this repo (use `--filter=blob:none` to save time) and checkout `pc_sampling_gfx1151` branch
2. `python setup_dkms_linked.py`
3. `sudo ln -s ~/amdgpu/amdgpu-dkms /usr/src/amdgpu-1.0`
4. `sudo dkms build --force amdgpu/1.0`
5. `sudo dkms install --force amdgpu/1.0`
6. Go on to install ROCr and ROCProfiler in my forked [rocm-systems](https://github.com/woct0rdho/rocm-systems)
