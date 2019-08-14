```
```
#
``` cameraPositionOptimization
```
Computes an optimizacion of the camera poses using CERES to solve a LMS problem

You need to install ceres from the repositoty and clone .h contained library directly in /usr/local/include
https://github.com/ryanhaining/cppitertools/blob/master/README.md#zip

All data is organized in these two structures:

A map indexed by camera names (as in innermodel_file.xml) and with the name of the parent transformation, the pose of the camera in the world reference system and a pointer to an array of doubles for the use of CERES in perturbing the 6 parameteres of the camera' pose. Final solution will be there.

std::map<std::string, std::tuple<std::string, QVec, double * >> cameras_map; 
        //cam : {t_cam, mutable_camera_for_CERES }

A list with the measurements read from the file: cam A, mark from the cam, cam B, mark from the cam B

std::list<std::tuple<std::string, QVec, std::string, QVec>> measurements; 
        //camA, mut indexA markA, camB, mut_indexB, markB

## Configuration parameters
As any other component,
``` *cameraPositionOptimization* ```
needs a configuration file to start. In

    etc/config

you can find an example of a configuration file. We can find there the following lines:

    EXAMPLE HERE


## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

    cd

``` <cameraPositionOptimization 's path> ```

    cp etc/config config

After editing the new config file we can run the component:

    bin/

```cameraPositionOptimization ```

    --Ice.Config=config
