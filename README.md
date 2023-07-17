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

# run opt_MFE() function in main_lander.py (the opt() function solving the problem just formulated by Simultaneous Approach)
# for better success rate of the solution，the solution process is divided into 2 steps.
#################### STEP 1 ###########################
# the problem with the following parameters and constraints is solved first.
# The following are the changs of the codes in opt_MFE() function 
$ initial_data_path = 'YOUR_PROJECT_ADRESS/EMFE_demo/output/Lander/opt_EMFE_2023_04_26_14_21_09' # You can use the opt() function in main_lander.py to calculate the initial data for EMFE and there is already an initial value in the /output/lander folder now
$ load_MFE_data_flag = False          # all finite element are equal and fixed
$ def add_numerical_error_constraints(self):
$   self.m.control_gradient_con = ConstraintList(rule=self._control_gradient_con(self.m, self.ncp))
$   self.m.noncollocation_point_error_con = ConstraintList(rule=self._noncollocation_point_error_con(self.m, self.ncp))
# the optimized result in folder output/Lander/opt_EMFE_2023_04_26_14_22_38_template
# In fact, the initial value given here is the solution result of EMFE, but the control sequence of the solution result loses the bang-bang characteristics, and the error of its non-collocation point error becomes larger
# the result of the opt() function should be used as the initial value, but in order to overcome the initial value sensitivity of the nonlinear optimization, the solution process is relatively complicated (adding constraints step by step until the nonlinear solution is successfully solved under complete constraints, mainly relying on experience). Here, for the convenience of demonstration, the result of EMFE is used as the initial value. 


#################### STEP 2 ###########################
# Then use the result of the problem just solved as the initial data to solve the EMFE problem
# The following are the changs of the codes in the opt_MFE() function 
$ initial_data_path = 'The address of the result folder of the just solved problem' #'YOUR_PROJECT_ADRESS/EMFE_demo/output/Lander/opt_EMFE_2023_04_26_14_22_38_template'
$ load_MFE_data_flag = True   # set finite element free
$ def add_numerical_error_constraints(self):
$   self.m.control_gradient_con = ConstraintList(rule=self._control_gradient_con(self.m, self.ncp))
$   self.m.noncollocation_point_error_con = ConstraintList(rule=self._noncollocation_point_error_con(self.m, self.ncp))
$   self.m.noncollocation_error_magcon = Constraint(rule=self._noncollocation_error_magcon)
# the optimized result in folder: output/Lander/opt_EMFE_2023_04_26_14_23_20
# the control sequence is bang-bang and non-collocation point error is minimized

```

## :memo: License ##

This project is under license from [MIT](https://opensource.org/licenses/MIT).

&#xa0;

<a href="#top">Back to top</a>
