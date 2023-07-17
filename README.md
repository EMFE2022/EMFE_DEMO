[//]: # (<div align="center" id="top"> )

[//]: # (  <img src="./.github/app.gif" alt="EMFE_demo" />)

[//]: # ()
[//]: # (  &#xa0;)

[//]: # ()
[//]: # (  <!-- <a href="https://emfe_demo.netlify.app">Demo</a> -->)

[//]: # (</div>)

<h1 align="center">EMFE_DEMO</h1>

<p align="center">
  <img alt="Github top language" src="https://img.shields.io/github/languages/top/{{EMFE2022}}/emfe_demo?color=56BEB8">

  <img alt="Github language count" src="https://img.shields.io/github/languages/count/{{EMFE2022}}/emfe_demo?color=56BEB8">

  <img alt="Repository size" src="https://img.shields.io/github/repo-size/{{EMFE2022}}/emfe_demo?color=56BEB8">

  <img alt="License" src="https://img.shields.io/github/license/{{EMFE2022}}/emfe_demo?color=56BEB8">

  <!-- <img alt="Github issues" src="https://img.shields.io/github/issues/{{EMFE2022}}/emfe_demo?color=56BEB8" /> -->

  <!-- <img alt="Github forks" src="https://img.shields.io/github/forks/{{EMFE2022}}/emfe_demo?color=56BEB8" /> -->

  <!-- <img alt="Github stars" src="https://img.shields.io/github/stars/{{EMFE2022}}/emfe_demo?color=56BEB8" /> -->
</p>

<!-- Status -->

<!-- <h4 align="center"> 
	🚧  EMFE_demo 🚀 Under construction...  🚧
</h4> 

<hr> -->

<p align="center">
  <a href="#dart-about">About</a> &#xa0; | &#xa0; 
  <a href="#sparkles-features">Features</a> &#xa0; | &#xa0;
  <a href="#white_check_mark-requirements">Requirements</a> &#xa0; | &#xa0;
  <a href="#checkered_flag-starting">Starting</a> &#xa0; | &#xa0;
  <a href="#memo-license">License</a> &#xa0; | &#xa0;
  <a href="https://github.com/{{EMFE2022}}" target="_blank">Author</a>
</p>

<br>

## :dart: About ##

This is the code for the paper [Enhanced moving finite element method based on error geometric estimation for simultaneous trajectory optimization](https://doi.org/10.1016/j.automatica.2022.110711)

## :sparkles: Features ##

:heavy_check_mark: Locate the jump points of the control profile;\
:heavy_check_mark: Obtain a local minimum non-collocation point error;\
:heavy_check_mark: The structure of the Hamilton function under the optimal condition remains unchanged;

## :white_check_mark: Requirements ##

The following tools were used in this project:

- [ipopt](https://coin-or.github.io/Ipopt/)(with [MA57 linear solver](https://www.hsl.rl.ac.uk/catalogue/ma57.html))
- [pyomo](https://github.com/Pyomo/pyomo)
- [numpy](https://numpy.org/)
- [scipy](https://scipy.org/)
- [pandas](https://pandas.pydata.org/)
- [matplotlib](https://matplotlib.org/)

## :checkered_flag: Starting ##

```bash

############################### NOTICE ###############################
# Due to the initial value sensitivity of nonlinear optimization, the current code has not yet realized the automatic solution process under the condition of unsatisfactory initial value

############################### INTRODUCTION ###############################
# Run opt_MFE() function in main_lander.py (the opt() function solving the problem just formulated by Simultaneous Approach, when run opt()，the initial value of opt() can be any one in the /output/Lander/ folder)
# For better success rate of the solution, the solution process is divided into 2 steps.

#################### STEP 1 ###########################
# The problem with the following parameters and constraints is solved first.
# The following are the changes to the codes in opt_MFE() function 
$ initial_data_path = 'YOUR_PROJECT_ADRESS/EMFE_demo/output/Lander/opt_EMFE_2023_04_26_14_21_09' # there is already an initial value in the /output/lander folder now
$ load_MFE_data_flag = False          # all finite element length (tf) are equal and fixed and intial value of noncollocation_error is zero
$ def add_numerical_error_constraints(self):
$   self.m.control_gradient_con = ConstraintList(rule=self._control_gradient_con(self.m, self.ncp))
$   self.m.noncollocation_point_error_con = ConstraintList(rule=self._noncollocation_point_error_con(self.m, self.ncp))
# The optimized result in folder output/Lander/opt_EMFE_2023_04_26_14_22_38_template

### TIPS ###
# 1) In order to reduce the initial value sensitivity, the initial value given here is the result of EMFE, otherwise the solution process may require a complex iterative process (presented in item 3)
# 2) If the initial value of the EMFE result is used, step 1 may be redundant, but it can more intuitively show the effect of the moving finite element. Compared with initial data, the result becomes just like solving the problem formulated by Simultaneous Approach (no bang-bang structure and large non-collocation-point error) except the effection of control_gradient_con. In addition, even if the  mu_list[1] is not zero, the non-collocation-point error is basically not optimized due to the fixed length of the finite element 
# 3) The result of the opt() function can be used as the initial value too, but due to initial value sensitivity, it may be necessary to add control_gradient_con and noncollocation_point_error_con gradually with mu_list[1]=0 and load_MFE_data_flag = False (this operation requires experience and hard to show here). In the above iterative process, the result of the previous iteration needs to be used as the initial value of the next iteration.

#################### STEP 2 ###########################
# Then use the result of the problem just solved as the initial data to solve the complete EMFE problem
# The following are the changes to the codes in the opt_MFE() function 
$ initial_data_path = 'The address of the result folder of the just solved problem' #'YOUR_PROJECT_ADRESS/EMFE_demo/output/Lander/opt_EMFE_2023_04_26_14_22_38_template'
$ load_MFE_data_flag = True             # set finite element length free and update inital value of noncollocation_error
$ def add_numerical_error_constraints(self):
$   self.m.control_gradient_con = ConstraintList(rule=self._control_gradient_con(self.m, self.ncp))
$   self.m.noncollocation_point_error_con = ConstraintList(rule=self._noncollocation_point_error_con(self.m, self.ncp))
$   self.m.noncollocation_error_magcon = Constraint(rule=self._noncollocation_error_magcon)
# The optimized result in folder: output/Lander/opt_EMFE_2023_04_26_14_23_20
# The control sequence is bang-bang and non-collocation-point error is minimized

```

## :memo: License ##

This project is under license from [MIT](https://opensource.org/licenses/MIT).

&#xa0;

<a href="#top">Back to top</a>
