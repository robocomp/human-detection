```
```
#
``` cameraPositionOptimization
```
Computes an optimizacion of the camera poses using CERES to solve a LMS problem

You need to install ceres from the repositoty and clone .h contained library directly in /usr/local/include
https://github.com/ryanhaining/cppitertools/blob/master/README.md#zip

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
