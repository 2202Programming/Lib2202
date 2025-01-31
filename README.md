# Lib2202
Common code across multiple years and robots.

To use this code set it should be added to your robot's project as a git
submodule.


 git submodule add https://github.com/2202Programming/Lib2202.git src\main\java\frc\lib2202


 This will create a .gitmodules in the parent's project file that looks something like this:

 [submodule "src/main/java/frc/lib2202"]
	path = src/main/java/frc/lib2202
	url = https://github.com/2202Programming/Lib2202.git


Robot code will then be based on both your project's repo and the library repo.
