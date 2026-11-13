# GRU-based Nonlinear Model Predictive Control (NMPC) for H2-Diesel Dual-Fuel Engines

[![DOI](https://zenodo.org/badge/1036037973.svg)](https://doi.org/10.5281/zenodo.16902939)

This repository contains MATLAB® and Simulink® code for generating a **Nonlinear Model Predictive Controller (NMPC)** based on a trained **GRU (Gated Recurrent Unit) Deep Neural Network** model.  
The DNN training scripts are available in a separate repository: [Zenodo Publication](https://doi.org/10.5281/zenodo.16902579).  

Authors:  
- **Alexander Winkler** (alexander.winkler@rwth-aachen.de)  
- Armin Norouzi (arminnorouzi2016@gmail.com)  
- David Gordon (dgordon@ualberta.ca)  
- Eugen Nuss (e.nuss@irt.rwth-aachen.de)  
- Vasu Sharma (vasu.sharma@rwth-aachen.de)  

---

## Repository Structure

- **`acados_implementation/`**  
  Contains MATLAB/Simulink code for generating and running the GRU-based NMPC controller using [acados](https://github.com/acados/acados).  
  The controller can be:
  - Run in standalone closed-loop simulation, or  
  - Deployed to embedded hardware (e.g., Raspberry Pi) using the acados embedded workflow.  

---

## MPC Controller (`acados_implementation/`)

1. Execute `setpath` (drag and drop into MATLAB).  
2. Check the `env_sh` call for correct **acados** paths.  
   - It must point to the **acados example MATLAB folder** to set environment variables correctly.  
3. Verify `matlab2tikz` path in `setpath`.  
4. In the code, set the **`no_fb` option** for an MPC variant independent of the IMEP feedback variable.  
5. Check the paths for code-generated files:  
   - See `make_sfun.m` for all required files.  
   - Copy libraries and includes from the **acados root folder** (ensure version compatibility).  
6. Execution options:  
   - **Standalone simulation**: run `sim_xx` scripts.  
   - **Embedded execution**: run `pi_xx` scripts for deployment on Raspberry Pi or similar devices.  
7. When using Simulink:  
   - Copy/paste the **acados include files** into the linked repository.  
   - Adjust **Simulink Strg+E (Configuration Parameters)** settings accordingly.  
   - Refer to the [acados embedded workflow guide](https://docs.acados.org/embedded_workflow/index.html) for details.  
8. Several plotting and evaluation options are available within the run script.  

---

## Controller settings used in the paper

The defaults in `test/run_SS_H2DF_control.m` and `nmpc/construct_C2C_NMPC_H2DF_LSTM_GRU.m` correspond to the **Core controller** of the paper (Tables A.2 and A.3). All quantities are normalised, so the weights are dimensionless.

| Cost term | Weight |
|---|---|
| Load tracking, p_mi | 1 |
| NOx, PM | 1e-3, 1e-3 |
| Maximum pressure-rise rate, dp_max | 0.1 |
| t_Main, t_P2M, alpha_Main, t_H2 | 6e-4, 1e-3, 1e-3, 1e-3 |
| Input rate, all four inputs | 0.1 |
| Slack, linear and quadratic | 10 |

Soft output limits (upper): p_mi 9 bar, NOx 1200 ppm, PM 1.5 mg/m3, dp_max 15 bar/CAD (lower limits 0).

**How the experiments were run.** On the test bench the cost weights and the output limits were not taken from this script: they were set at run time through the inputs of the acados Simulink block (`cost_W`, `lh`/`uh`, `lbx`/`ubx`), which allowed the controller to be tuned on the running engine without regenerating the solver. The applied weights were logged with the measurements (`C2C_NMPC_weights_*`). The slack weights are not a run-time input of the Simulink block in the acados version used and were set in `construct_C2C_NMPC_H2DF_LSTM_GRU.m` before code generation. For the low- and high-H2 settings of the paper, only the weights on t_Main and t_H2 were changed at run time.

acados scales all quadratic cost terms (including the quadratic slack weight) by 1/2 but not the linear slack term.

---

## Changelog

- **v1.1**: default weights and soft limits set to the Core controller of the paper (p_mi weight 3 -> 1; slack weights 1 -> 10; soft limits 9.5 bar / 1200 ppm / 0.8 mg/m3 / 12 bar/CAD -> 9 bar / 1200 ppm / 1.5 mg/m3 / 15 bar/CAD); README section on the settings used in the experiments; citation and contact details corrected.
- **v1.0**: initial release. The defaults reflected an earlier tuning stage, not the values applied on the test bench.

---

## Dependencies

The code runs on **MATLAB R2024a or newer** and requires:  
- [matlab2tikz](https://github.com/matlab2tikz/matlab2tikz)  
- LaTeX `tikz` package (also available as a MATLAB add-on)  
- [acados](https://github.com/acados/acados) (installed properly and configured with MATLAB interface)  

---

## Cite us

If you are using this code, please cite the following publications:  
- A. Winkler, V. Sharma, J. Bedei, E. Sperling, C. R. Koch, D. Gordon, J. Andert: *Hydrogen-Diesel Dual-Fuel Engine Operation Using Real-Time GRU-Based Nonlinear Model Predictive Control*, preprint, arXiv:2609.15819.
- The DNN training software and measurement data on **Zenodo**: [https://doi.org/10.5281/zenodo.16902579](https://doi.org/10.5281/zenodo.16902579)
- This software: [https://doi.org/10.5281/zenodo.16902939](https://doi.org/10.5281/zenodo.16902939)


---

## License

This project is licensed under the  
[Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0.txt).
