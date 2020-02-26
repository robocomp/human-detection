```
```
#
``` humanintegration
```
This code has to be installed. Include files and static lib have to be copied to component's src directory
https://www.codeproject.com/Articles/20027/JSON-Spirit-A-C-JSON-Parser-Generator-Implemented


## Configuration parameters
As any other component,
``` *humanintegration* ```
needs a configuration file to start. In

    etc/config

you can find an example of a configuration file. We can find there the following lines:

    EXAMPLE HERE


## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

    cd

``` <humanintegration 's path> ```

    cp etc/config config

After editing the new config file we can run the component:

    bin/

```humanintegration ```

    --Ice.Config=config
