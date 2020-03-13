
# Continuous Delivery
Once your ROS package is built, it needs to be deployed to customers. ROS on Windows leverages the Chocolatey Package manager for Delivery.

Chocolatey is an Open Source package manager for Windows, with a command line interface. Chocolatey packages are zip files which contain a descriptor - based on the Nuget Library manager. Once created, Packages are published on the Chocolatey package registry. 

## Sign up to Publish packages
Before you can publish packages, you need to acquire an API key from Chocolatey.
To acquire an API key, youneed to sign up for an account. 

You can associate the API Key with your chocolatey command line tool to publish with the following command line:
```
choco apikey --key xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxxxxxx --source https://push.chocolatey.org/
```

## Packaging collateral
To build a chocolatey package, the following files and structure is required in your repository.

  * Repository Root
    * package
      * `package-build.yaml`
      * `MyPackage.nuspec`
      * `build.bat`
      * `tools`
        * `chocolateyInstall.ps1`
        * `rosdepInstall.bat`


## Crafting a chocolatey package
A Chocolatey package includes a xml descriptor of the package. 

Create a file named after your package, with the contents of [template.nuspec](choco_template.md).

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
    <iconUrl></iconUrl>
    <requireLicenseAcceptance>false</requireLicenseAcceptance>
    <description>MyCoolRobot for ROS on Windows.</description>
    <summary>MyCoolRobot packaged as a rosdep.</summary>
    <tags>MyCoolRobot</tags>
    <packageSourceUrl>http://www.github.com/ContosoRobotics/MyCoolRobot</packageSourceUrl>
    <docsUrl>http://www.github.com/ContosoRobotics/MyCoolRobot/Docs</docsUrl>
  </metadata>
</package>
```

## Adding install scripts
In the tools folder, create the setup files. You can use these directly, or customize them for your project. Examples of customization include custom dependencies (like fetching and installing external Msi files).

Place these files into the tools folder.

[chocolateyInstall.ps1](chocolateyInstall.md)

[rosdepInstall.bat](rosdepInstall.bat)

## Adding the build script
In the root of the package folder, add [build.bat](choco_build.md). Replace `<ros package>` with the name of your nuspec from above.

## Setting up the pipeline
In the package folder, add [package-build.yaml](choco_pipeline_package.md). 

In your Azure Pipelines or Github Workflow, add a referene to this file before your test entries;

```yaml
  steps:
  - template: ..\package\package-build.yml
```

## Manually Publish a chocolatey package
```
choco push MyPackage.1.0.nupkg --source https://push.chocolatey.org/
```
## Automatically Publishing chocolatey package
