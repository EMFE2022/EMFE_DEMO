[//]: # (<div align="center" id="top"> )

[//]: # (  <img src="./.github/app.gif" alt="EMFE_demo" />)

[//]: # ()
[//]: # (  &#xa0;)

[//]: # ()
[//]: # (  <!-- <a href="https://emfe_demo.netlify.app">Demo</a> -->)

[//]: # (</div>)

<h1 align="center">EMFE_DEMO</h1>

<p align="center">
  <img alt="Github top language" src="https://img.shields.io/github/languages/top/{{YOUR_GITHUB_USERNAME}}/emfe_demo?color=56BEB8">

  <img alt="Github language count" src="https://img.shields.io/github/languages/count/{{YOUR_GITHUB_USERNAME}}/emfe_demo?color=56BEB8">

  <img alt="Repository size" src="https://img.shields.io/github/repo-size/{{YOUR_GITHUB_USERNAME}}/emfe_demo?color=56BEB8">

  <img alt="License" src="https://img.shields.io/github/license/{{YOUR_GITHUB_USERNAME}}/emfe_demo?color=56BEB8">

  <!-- <img alt="Github issues" src="https://img.shields.io/github/issues/{{YOUR_GITHUB_USERNAME}}/emfe_demo?color=56BEB8" /> -->

  <!-- <img alt="Github forks" src="https://img.shields.io/github/forks/{{YOUR_GITHUB_USERNAME}}/emfe_demo?color=56BEB8" /> -->

  <!-- <img alt="Github stars" src="https://img.shields.io/github/stars/{{YOUR_GITHUB_USERNAME}}/emfe_demo?color=56BEB8" /> -->
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
  <a href="https://github.com/{{YOUR_GITHUB_USERNAME}}" target="_blank">Author</a>
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

- [ipopt](https://coin-or.github.io/Ipopt/)
- [pyomo](https://github.com/Pyomo/pyomo)
- [numpy](https://numpy.org/)
- [scipy](https://scipy.org/)
- [pandas](https://pandas.pydata.org/)
- [matplotlib](https://matplotlib.org/)

## :checkered_flag: Starting ##

```bash

# When using EMFE, in order to ensure the success rate of the solution，the problem with the following parameters and constraints is solved first.
# parameters (in opt_MFE())
$ initial_data_path = 'YOUR_PROJECT_ADRESS/EMFE_demo/output/Lander/opt_EMFE_2023_04_26_14_21_09' ## You can use the opt() function in main_lander.py to calculate the initial data for EMFE and there is already an initial value in the /output/lander folder now
$ load_MFE_data_flag = False
# constraints (Lunar_lander_3DOF_MFE_model.py)
$ def add_numerical_error_constraints(self):
$   self.m.control_gradient_con = ConstraintList(rule=self._control_gradient_con(self.m, self.ncp))
$   self.m.noncollocation_point_error_con = ConstraintList(rule=self._noncollocation_point_error_con(self.m, self.ncp))

#Then use the result of the problem just solved as the initial data to solve the complete EMFE problem
# parameters (in opt_MFE())
$ initial_data_path = 'The address of the result folder of the just solved problem' #'YOUR_PROJECT_ADRESS/EMFE_demo/output/Lander/opt_EMFE_2023_04_26_14_22_38_template'
$ load_MFE_data_flag = True
# constraints (Lunar_lander_3DOF_MFE_model.py)
$ def add_numerical_error_constraints(self):
$   self.m.control_gradient_con = ConstraintList(rule=self._control_gradient_con(self.m, self.ncp))
$   self.m.noncollocation_point_error_con = ConstraintList(rule=self._noncollocation_point_error_con(self.m, self.ncp))
$   self.m.noncollocation_error_magcon = Constraint(rule=self._noncollocation_error_magcon)


```

## :memo: License ##

This project is under license from [MIT](https://opensource.org/licenses/MIT).

&#xa0;

<a href="#top">Back to top</a>
