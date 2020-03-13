```yaml
parameters:
  PACKAGE_BUILD_WORKING_DIRECTORY: $(Build.SourcesDirectory)\package-build

steps:
- task: ArchiveFiles@2
  inputs:
    rootFolderOrFile: 'C:\opt\ros'
    includeRootFolder: false
    archiveFile: ${{ parameters.PACKAGE_BUILD_WORKING_DIRECTORY }}\tools\drop.zip
  displayName: 'Archive ROS binaries'
- script: |
    call "build.bat"
  workingDirectory: ${{ parameters.PACKAGE_BUILD_WORKING_DIRECTORY }}
  displayName: 'Build ROS packages'
- task: PublishBuildArtifacts@1
  inputs:
    pathtoPublish: ${{ parameters.PACKAGE_BUILD_WORKING_DIRECTORY }}\output
    artifactName: 'package-build-drop'
  displayName: 'Publish ROS packages'
- task: PublishBuildArtifacts@1
  inputs:
    pathtoPublish: ${{ parameters.PACKAGE_BUILD_WORKING_DIRECTORY }}\output-pre
    artifactName: 'package-build-drop-prerelease'
  displayName: 'Publish ROS packages'
```