tmc_wrs_binary
---------------

Authors
---------------
 * Yosuke Matsusaka

Contact
---------------
 * HSR Support <xr-hsr-support@mail.toyota.co.jp>

LICENSE
---------------
This software is released under the BSD 3-Clause Clear License, see LICENSE.txt.


How to build the docker image.
---------------
1. Pull the repos using [vcstool](https://github.com/dirk-thomas/vcstool)

  ```
  vcs import < tiago_minimal.repos
  ```

2. Build the image

 ```
  ./build.sh
 ```
 
3. (Optional) Push the image to github (Requires to configure your account) 

 ```
  ./push.sh
 ```
 
 How to build your own docker image.
---------------
1. Modify [tiago_minimal.repos](https://github.com/gentlebots/tmc_wrs_binary/blob/master/tiago_minimal.repos) file adding your own repos.

2. Pull the repos using [vcstool](https://github.com/dirk-thomas/vcstool)

  ```
  vcs import < tiago_minimal.repos
  ```
3. Edit the [entrypoint](https://github.com/gentlebots/tmc_wrs_binary/blob/master/supervisord.conf#L21) to execute a different launcher.
4. Build the image

 ```
  ./build.sh
 ```
 
5. (Optional) Push the image to github (Requires to configure your account) 

 ```
  ./push
