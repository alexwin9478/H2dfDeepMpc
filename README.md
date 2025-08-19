# GRU-based Nonlinear Model Predictive Control (NMPC) for H2-Diesel Dual-Fuel Engines

[![DOI](https://zenodo.org/badge/1036036262.svg)](https://doi.org/10.5281/zenodo.16902939)

This repository contains MATLAB® and Simulink® code for generating a **Nonlinear Model Predictive Controller (NMPC)** based on a trained **GRU (Gated Recurrent Unit) Deep Neural Network** model.  
The DNN training scripts are available in a separate repository: [Link].  

Authors:  
- **Alexander Winkler** (alexander.winkler@rwth-aachen.de)  
- Armin Norouzi (arminnorouzi2016@gmail.com)  
- David Gordon (dgordon@ualberta.ca)  
- Eugen Nuss (e.nuss@irt.rwth-aachen.de)  
- Vasu Sharma (alexander.winkler@rwth-aachen.de) 

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

## Dependencies

The code runs on **MATLAB R2024a or newer** and requires:  
- [matlab2tikz](https://github.com/matlab2tikz/matlab2tikz)  
- LaTeX `tikz` package (also available as a MATLAB add-on)  
- [acados](https://github.com/acados/acados) (installed properly and configured with MATLAB interface)  

---

## Cite us

If you are using this code, please cite the following publications:  
- [Dummy1] Paper 1, tbd  
- [Dummy2] Paper 2, tbd 
- The data publication on **Zenodo**:
[![DOI](https://zenodo.org/badge/1036036262.svg)](https://doi.org/10.5281/zenodo.16902579)


---

## License

This project is licensed under the  
[Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0.txt).
