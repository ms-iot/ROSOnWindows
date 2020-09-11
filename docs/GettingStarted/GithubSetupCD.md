
# Continuous Delivery
Once your ROS package is built, it needs to be deployed to customers. ROS on Windows leverages the Chocolatey Package manager for Delivery.

Chocolatey is an Open Source package manager for Windows, with a command line interface. Chocolatey packages are zip files which contain a descriptor - based on the Nuget Library manager. Once created, Packages are published to the Chocolatey package registry. 

## Sign up to Publish packages
Before you can publish packages, you need to acquire an API key from Chocolatey.
To acquire an API key, you need to [register for an account](https://chocolatey.org/account/Register). Once you've registered for an account, you'll be assigned an API key, which you can assign to your local chocolatey install - and later as a Pipeline Secret.

You can associate the API Key with your chocolatey command line tool to publish with the following command line:
```
choco apikey --key xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxxxxxx --source https://push.chocolatey.org/
```

## Packaging collateral
To build a chocolatey package, the following files and structure is recommended in your repository. The files will be described later.

* Repository Root
    * `package`
        * `MyPackage.nuspec`
        * `build.bat`
        * `chocolateyInstall_template.ps1`
        * `tools`
            * `rosdepInstall.bat`


## Crafting a `MyPackage.nuspec` file
A Chocolatey package includes a xml descriptor of the package. Create a file in the `package` folder named after your ROS package, with the code contents of [template.nuspec](choco_template.md).

For example, if your ROS package is called MyCoolRobot, the file should be called `MyCoolRobot.nuspec`

Modify the properties of the XML file based on your specific requirements. For example:

```xml
<?xml version="1.0"?>
<package xmlns="http://schemas.microsoft.com/packaging/2010/07/nuspec.xsd">
  <metadata>
    <id>MyCoolRobot</id>
    <version>1.0.0</version>
    <title>MyCoolRobot ROS package</title>
    <authors>Malcom Reynolds</authors>
    <licenseUrl>http://www.github.com/ContosoRobotics/MyCoolRobot/License.txt</licenseUrl>
    <projectUrl>http://www.github.com/ContosoRobotics/MyCoolRobot</projectUrl>
    <requireLicenseAcceptance>false</requireLicenseAcceptance>
    <description>MyCoolRobot for ROS on Windows.</description>
    <summary>MyCoolRobot packaged as a rosdep.</summary>
    <tags>ros MyCoolRobot</tags>
    <packageSourceUrl>http://www.github.com/ContosoRobotics/MyCoolRobot</packageSourceUrl>
    <docsUrl>http://www.github.com/ContosoRobotics/MyCoolRobot/Docs</docsUrl>
  </metadata>
</package>
```

## Adding install scripts
Create these files in the tools folder, and add the code contents of each. You can use these directly, or customize them for your project. Examples of customization include custom dependencies (like fetching and installing external Msi files).

* [chocolateyInstall_template.ps1](chocolateyInstall_template.md)
* tools
    * [rosdepInstall.bat](rosdepInstall.md)

## Adding the build script
In the root of the package folder, add a file with the code contents of [build.bat](choco_build.md). Replace `<ros package>` with the name of your nuspec from above.

## Testing the package
In later states, the pipelines will generate a zip file containing the output of your ROS package. 

To simulate archive generation for testing, in your terminal window, change directory into the workspace containing your package, then build with install target.

Install 7zip using Chocolatey (You only need to do this the first time you try to build a package).

```batch
choco install 7zip
```

**Catkin**
```batch
catkin_make install -DCATKIN_BUILD_BINARY_PACKAGE=ON
```

**Colcon**

*Coming Soon*

### Package

```batch
cd install
7z a -tzip ..\src\<ros package>\package\tools\drop.zip *
cd ..\src\<ros package>\package
build.bat
```
If this command succeeds, you will have two nupkg files - one for pre-release in `output-pre`, and one release package in `output`.

Install using the following command:
```batch
choco install output-pre\<ros package>.1.0.0.nupkg
```

**Troubleshoot**
If the package fails to install correctly, the nupkg is a zip file. You can expand the zip file and examine the contents. Verify that names are spelled correctly and that the contents match. Once you find the error, you can recreate the package using build.bat, and reinstall using:

```batch
choco install output-pre\<ros package>.1.0.0.nupkg --force
```

> NOTE: Chocolatey will remove the root folder of a zip based chocolatey package - which potentially includes other zip based packages. We recommend not uninstalling a chocolatey package for this reason.

## Manually Publish a chocolatey package
```
choco push MyPackage.1.0.0.nupkg --source https://push.chocolatey.org/
```

# Automatically generate chocolatey package during CI
Once you've crafted your nuspec and tested the installation, you can generate the chocolatey package during CI and publish it as a release. 

Now create the Github Action

  * On github, select the `Action` tab.
  * Create a new workflow
  * In the new workflow, copy the contents of [CD.yaml](ros1_workflow_pub.md) to the newly created workflow file.
  * Replace `<ros package>` with the ROS package you are generating
  * Replace `<ros nuspec>` with the name of the nuspec you from above
  * Replace `<ros version>` with the version from your nuspec.

Whenever you create a tag in the repo, it will trigger the deployment workflow, which will publish the nupkg as a github release.  

### Updating Chocolatey rosdep mappings
Once your chocolatey package has been published, `rosdep` needs to be informed of how to find it. `rosdep` enumerates entries in `package.xml`, then uses a yaml mapping file to locate the package. 

To update that mapping file, please follow these steps:

* Fork [https://github.com/ms-iot/rosdistro-db](https://github.com/ms-iot/rosdistro-db) &nearr; into your github account
* Create a file called `0-update.list` in `c:\opt\ros\melodic\x64\etc\ros\rosdep\sources.list.d`
* In this file, add a line which points to your fork:
```bat
# os-specific listings first
yaml https://raw.githubusercontent.com/<your github>/rosdistro-db/init_windows/rosdep/win-chocolatey.yaml windows
yaml https://raw.githubusercontent.com/<your github>/rosdistro-db/init_windows/rosdep/vcpkg.yaml windows
```
* Add a mapping from the dependency name used in the ROS package



`Python`
```bat
<python-package-name>:
    windows:
      pip:
        packages: [<python-package-name-in-pip>]
```
  `C++`
```bat
  <package-name>:
    windows:
      chocolatey:
        depends: [<chocolatey dependencies which aren't specified in the package.xml>]
        packages: [<chocolatey-name>]
```

* Update rosdeps on your computer.
```bat
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

> You may encounder a warning about missing packages. On Windows some packages were collapsed into their metapackage hosting package due to differences in dependency behavior on Windows.

