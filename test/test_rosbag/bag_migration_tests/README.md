# Bag migration tests scripts

In order to generate the data and rules for bag migration tests, two scripts are provided:

 * **generate_data** : Generates *.bag* files with messages (data) to run the bag migration tests. 
 * **generate_rules** : Help in the process of creating rules between two message generations.

The next sections explain how to use these scripts in detail. The rest are auxiliary scripts that should NOT be used directly.

Before running any of these scripts you need to build the **test_rosbag** package at least once in your workspace and `source` the environment workspace setup file. This can be done running the following from the root of the workspace:

``` bash
$ catkin_make_isolated
$ source devel_isolated/test_rosbag/setup.bash
```

:warning: Since we use `catkin_make_isolated` in the scripts to build the different message generations, remember the scripts need to be run from the root of the workspace.

## Generate Data

The **generate_data** script generates *.bag* data files for one or multiple messsage generations. The output directory is the **bag_migration_tests/test** folder inside the **test_rosbag** package. It's the user responsability to move the output *.bag* files to an HTTP server and download them from the **CMakeLists.txt** file as in this example:

``` cmake
catkin_download_test_data(download_data_test_constants_gen1.bag http://download.ros.org/data/test_rosbag/constants_gen1.bag FILENAME test/constants_gen1.bag MD5 77ec8cb20e823ee3f3a87d07ea1132df )
```

Currently, there are the following message generations on the **test_rosbag** package, under the **bag_migration_tests** folder, in these sub-folders:

* **msg_gen1** : Message generation \#1.
* **msg_gen2** : Message generation \#2.
* **msg_gen3** : Message generation \#3.
* **msg_current** : Current message generation.

:warning: Note the **generate_data** script doesn't generate data for the **msg_current** generation because we don't have to. This is because the current generation is never the source of any migration rules, just the target for the latest generation.

This script takes the following positional arguments:

* **generations** : ID of the messages generations to build and generate *.bag* data files for. It defaults to `1 2 3`.

### Examples

1. Generate data for the message generation \#1:

``` bash
$ rosrun test_rosbag generate_data 1
Generating data in '/home/eperdomo/dev/ws/clearpath_ws/src/ros_comm/test/test_rosbag/bag_migration_tests' for generation: 1
```

This will create the following *.bag* files inside the **test_rosbag** package:

``` bash
bag_migration_tests/test/constants_gen1.bag
bag_migration_tests/test/converged_gen1.bag
bag_migration_tests/test/convergent_gen1.bag
bag_migration_tests/test/migrated_addsub_gen1.bag
bag_migration_tests/test/migrated_explicit_gen1.bag
bag_migration_tests/test/migrated_implicit_gen1.bag
bag_migration_tests/test/migrated_mixed_gen1.bag
bag_migration_tests/test/partially_migrated_gen1.bag
bag_migration_tests/test/renamed_gen1.bag
bag_migration_tests/test/subunmigrated_gen1.bag
bag_migration_tests/test/unmigrated_gen1.bag
```

Each of these bag files have a single message of the generation requested, as shown below with `rosbag info`:

``` bash
$ rosbag info $(rospack find test_rosbag)/bag_migration_tests/test/constants_gen1.bag
path:        /home/eperdomo/dev/ws/clearpath_ws/src/ros_comm/test/test_rosbag/bag_migration_tests/test/constants_gen1.bag
version:     2.0
duration:    0.0s
start:       Dec 31 1969 19:00:00.00 (0.00)
end:         Dec 31 1969 19:00:00.00 (0.00)
size:        4.7 KB
messages:    1
compression: none [1/1 chunks]
types:       test_rosbag/Constants [06a34bda7d4ea2950ab952e89ca35d7a]
topics:      constants   1 msg     : test_rosbag/Constants
```

2. Generate data for the message generations \#2 and \#3:

``` bash
$ rosrun test_rosbag generate_data 2 3
Generating data in '/home/eperdomo/dev/ws/clearpath_ws/src/ros_comm/test/test_rosbag/bag_migration_tests' for generation: 2
Generating data in '/home/eperdomo/dev/ws/clearpath_ws/src/ros_comm/test/test_rosbag/bag_migration_tests' for generation: 3
```

This will create the following *.bag* files inside the **test_rosbag** package:

``` bash
bag_migration_tests/test/constants_gen2.bag
bag_migration_tests/test/converged_gen2.bag
bag_migration_tests/test/converged_gen3.bag
bag_migration_tests/test/convergent_gen2.bag
bag_migration_tests/test/migrated_explicit_gen2.bag
bag_migration_tests/test/migrated_explicit_gen3.bag
bag_migration_tests/test/migrated_implicit_gen2.bag
bag_migration_tests/test/migrated_implicit_gen3.bag
bag_migration_tests/test/migrated_mixed_gen2.bag
bag_migration_tests/test/migrated_mixed_gen3.bag
bag_migration_tests/test/partially_migrated_gen2.bag
bag_migration_tests/test/partially_migrated_gen3.bag
bag_migration_tests/test/renamed_gen2.bag
bag_migration_tests/test/renamed_gen3.bag
```

3. Generate data for all the message generations, relying on the default value for the argument:

``` bash
$ rosrun test_rosbag generate_data
Generating data in '/home/eperdomo/dev/ws/clearpath_ws/src/ros_comm/test/test_rosbag/bag_migration_tests' for generation: 1
Generating data in '/home/eperdomo/dev/ws/clearpath_ws/src/ros_comm/test/test_rosbag/bag_migration_tests' for generation: 2
Generating data in '/home/eperdomo/dev/ws/clearpath_ws/src/ros_comm/test/test_rosbag/bag_migration_tests' for generation: 3
```

This will create the following *.bag* files inside the **test_rosbag** package:

``` bash
bag_migration_tests/test/constants_gen1.bag
bag_migration_tests/test/constants_gen2.bag
bag_migration_tests/test/converged_gen1.bag
bag_migration_tests/test/converged_gen2.bag
bag_migration_tests/test/converged_gen3.bag
bag_migration_tests/test/convergent_gen1.bag
bag_migration_tests/test/convergent_gen2.bag
bag_migration_tests/test/migrated_addsub_gen1.bag
bag_migration_tests/test/migrated_explicit_gen1.bag
bag_migration_tests/test/migrated_explicit_gen2.bag
bag_migration_tests/test/migrated_explicit_gen3.bag
bag_migration_tests/test/migrated_implicit_gen1.bag
bag_migration_tests/test/migrated_implicit_gen2.bag
bag_migration_tests/test/migrated_implicit_gen3.bag
bag_migration_tests/test/migrated_mixed_gen1.bag
bag_migration_tests/test/migrated_mixed_gen2.bag
bag_migration_tests/test/migrated_mixed_gen3.bag
bag_migration_tests/test/partially_migrated_gen1.bag
bag_migration_tests/test/partially_migrated_gen2.bag
bag_migration_tests/test/partially_migrated_gen3.bag
bag_migration_tests/test/renamed_gen1.bag
bag_migration_tests/test/renamed_gen2.bag
bag_migration_tests/test/renamed_gen3.bag
bag_migration_tests/test/subunmigrated_gen1.bag
bag_migration_tests/test/unmigrated_gen1.bag
```

## Generate Rules

The **generate_rules** script helps in the process of creating migration rules from one message generation to another. The scripts requires the user input every time a rule is created, since the rule **update** method needs to be implemented and the rule needs to be set to **valid = True**. It also needs the user to tell the new name for moved/renamed messages. The user can also decide to completely remove the rule if the intention is to test what happens when rules are missed.

For convenience, the script loads all the migration rules (*.bmr* files) already available on the **msg_** and **test** sub-folders, under the **bag_migration_tests** folder in the **test_rosbag** package.

This script takes the following positional arguments:

* **source** : Source message generation.
* **target** : Target message generation.

The message generation can be any of the following: `gen1`, `gen2`, `gen3` or `current`. That is, the suffix of all the message generations available in the **test_rosbag** package, under the **bag_migration_tests** folder, with the **msg_** prefix.

The output folder for the generated rules is the folder for the **source** message generation, e.g. **msg_gen1** for `gen1`.

The script proceeds as follows:

1. Builds the **source** message generation.
2. Save the **source** full text message definition for all messages in this generation.
3. Builds the **target** message generation.
4. Creates all migration rules needed from **source** to **target** and prompts the user to implement them (or remove if not needed).

:warning: The migration rules needs to be organized after, since they can have misleading filenames.

### Examples

1. Generate migration rules from the generation \#1 to \#2 from scratch:

Since the **generate_rules** scripts loads the existing rules in the **test** sub-folder, we need to remove or rename them first:

``` bash
$ roscd test_rosbag/bag_migration_tests/test
$ ls *.bmr | xargs -I f mv f f.bak
$ cd -
```

Now we can generate the rules from scratch, without any other rules interfering with the new ones:

``` bash
$ rosrun test_rosbag generate_rules gen1 gen2
Building generation: gen1

Saving messages full text for generation: gen1
Saved message test_rosbag/SimpleMigrated: msg_gen1/SimpleMigrated.saved
Saved message test_rosbag/SubUnmigrated: msg_gen1/SubUnmigrated.saved
Saved message test_rosbag/Convergent: msg_gen1/Convergent.saved
Saved message test_rosbag/Unmigrated: msg_gen1/Unmigrated.saved
Saved message test_rosbag/MigratedImplicit: msg_gen1/MigratedImplicit.saved
Saved message test_rosbag/PartiallyMigrated: msg_gen1/PartiallyMigrated.saved
Saved message test_rosbag/Constants: msg_gen1/Constants.saved
Saved message test_rosbag/MigratedExplicit: msg_gen1/MigratedExplicit.saved
Saved message test_rosbag/Converged: msg_gen1/Converged.saved
Saved message test_rosbag/Simple: msg_gen1/Simple.saved
Saved message test_rosbag/MigratedMixed: msg_gen1/MigratedMixed.saved
Saved message test_rosbag/Renamed1: msg_gen1/Renamed1.saved
Saved message test_rosbag/MigratedAddSub: msg_gen1/MigratedAddSub.saved

Building generation: gen2

Making rules from source generation 'gen1' to target generation 'gen2'
WARNING: Within rule [GENERATED.update_test_rosbag_MigratedImplicit_e809c9d02226cf20287aad007a48a177] cannot migrate from subtype [MigratedExplicit] to [MigratedExplicit]..
The following migrations need to occur:
 * From: test_rosbag/MigratedImplicit [e809c9d02226cf20287aad007a48a177]
   To:   test_rosbag/MigratedImplicit [24b6ff9552f4ef454cf6b71bd64da259]
    1 rules missing:
     * From: test_rosbag/MigratedExplicit [c7936d50a749a1f589d6fc81eae24b34]
       To:   test_rosbag/MigratedExplicit [fa072fb3cfcf105e1e5609a7467e2a14]

The necessary rule files have been written to: msg_gen1/MigratedImplicit.bmr
Created rule for msg_gen1/MigratedImplicit.saved: msg_gen1/MigratedImplicit.bmr
Set valid = True, the order and implement the update method, or remove it if you don't want it.
Then press ENTER.

The following migrations need to occur:
 * From: test_rosbag/MigratedExplicit [c7936d50a749a1f589d6fc81eae24b34]
   To:   test_rosbag/MigratedExplicit [fa072fb3cfcf105e1e5609a7467e2a14]

No additional rule files needed to be generated.  msg_gen1/MigratedExplicit.bmr not created.
No rule needed for msg_gen1/MigratedExplicit.saved!
The following migrations need to occur:
 * From: test_rosbag/PartiallyMigrated [44a0fba96e9fa39d652ea664fc598253]
   To:   test_rosbag/PartiallyMigrated [e6d73341e7b3f15f987c0cf194f97350]

No additional rule files needed to be generated.  msg_gen1/PartiallyMigrated.bmr not created.
No rule needed for msg_gen1/PartiallyMigrated.saved!
Saved definition is up to date.
No rule needed for msg_gen1/SubUnmigrated.saved!
The following migrations need to occur:
 * From: test_rosbag/Renamed1 [2fbee7c2602a76620804dfad673383b9]
   To:   Unknown
    1 rules missing:
     * From: test_rosbag/Renamed1 [2fbee7c2602a76620804dfad673383b9]
       To:   Unknown
The message type test_rosbag/Renamed1 appears to have moved.  Please enter the type to migrate it to.
>test_rosbag/Renamed2
Creating the migration rule for test_rosbag/Renamed2 requires additional missing rules:

The necessary rule files have been written to: msg_gen1/Renamed1.bmr
Created rule for msg_gen1/Renamed1.saved: msg_gen1/Renamed1.bmr
Set valid = True, the order and implement the update method, or remove it if you don't want it.
Then press ENTER.

The following migrations need to occur:
 * From: test_rosbag/MigratedMixed [4517becf622c1e131ed4f7bfc3d5d093]
   To:   test_rosbag/MigratedMixed [5568494133a082ad58ef1aa391f47e2b]

No additional rule files needed to be generated.  msg_gen1/MigratedMixed.bmr not created.
No rule needed for msg_gen1/MigratedMixed.saved!
Saved definition is up to date.
No rule needed for msg_gen1/Simple.saved!
The following migrations need to occur:
 * From: test_rosbag/SimpleMigrated [f3d103d10e4d7f4e5c4b19aa46d9a9dd]
   To:   test_rosbag/SimpleMigrated [01dfc3630b3a6d483b2f36047889c82c]
    1 rules missing:
     * From: test_rosbag/SimpleMigrated [f3d103d10e4d7f4e5c4b19aa46d9a9dd]
       To:   test_rosbag/SimpleMigrated [01dfc3630b3a6d483b2f36047889c82c]

The necessary rule files have been written to: msg_gen1/SimpleMigrated.bmr
Created rule for msg_gen1/SimpleMigrated.saved: msg_gen1/SimpleMigrated.bmr
Set valid = True, the order and implement the update method, or remove it if you don't want it.
Then press ENTER.

The following migrations need to occur:
 * From: test_rosbag/Convergent [60346ca10167913d79a124d3920677c6]
   To:   test_rosbag/Convergent [fadc2231052dc3e2ecb5a84eea7e1e2d]

No additional rule files needed to be generated.  msg_gen1/Convergent.bmr not created.
No rule needed for msg_gen1/Convergent.saved!
The following migrations need to occur:
 * From: test_rosbag/MigratedAddSub [c5d68a52143661d05909248952fc11f1]
   To:   test_rosbag/MigratedAddSub [f54ff3b9ba622359fa96ac15d4498153]
    1 rules missing:
     * From: test_rosbag/MigratedAddSub [c5d68a52143661d05909248952fc11f1]
       To:   test_rosbag/MigratedAddSub [f54ff3b9ba622359fa96ac15d4498153]

The necessary rule files have been written to: msg_gen1/MigratedAddSub.bmr
Created rule for msg_gen1/MigratedAddSub.saved: msg_gen1/MigratedAddSub.bmr
Set valid = True, the order and implement the update method, or remove it if you don't want it.
Then press ENTER.

Saved definition is up to date.
No rule needed for msg_gen1/Unmigrated.saved!
The following migrations need to occur:
 * From: test_rosbag/Converged [13459e473cfc2d719fd495bd7a0b747b]
   To:   test_rosbag/Converged [a47f409894343606276bb95cd2fd6b12]

No additional rule files needed to be generated.  msg_gen1/Converged.bmr not created.
No rule needed for msg_gen1/Converged.saved!
The following migrations need to occur:
 * From: test_rosbag/Constants [06a34bda7d4ea2950ab952e89ca35d7a]
   To:   test_rosbag/Constants [ae89f5ac418be503530c99f2c7907db7]
    1 rules missing:
     * From: test_rosbag/Constants [06a34bda7d4ea2950ab952e89ca35d7a]
       To:   test_rosbag/Constants [ae89f5ac418be503530c99f2c7907db7]

The necessary rule files have been written to: msg_gen1/Constants.bmr
Created rule for msg_gen1/Constants.saved: msg_gen1/Constants.bmr
Set valid = True, the order and implement the update method, or remove it if you don't want it.
Then press ENTER.

* Remember that migration rules for renamed messages needs to be created manually!
* Also remember that the migration rules in the files generated by this script might belong to other messages,
  other than the one on the file name, so you should organize them properly!
```

This will create the following *.saved* full text message definitions on the **source** folder, i.e. **msg_gen1**:

``` bash
bag_migration_tests/msg_gen1/Constants.saved
bag_migration_tests/msg_gen1/Converged.saved
bag_migration_tests/msg_gen1/Convergent.saved
bag_migration_tests/msg_gen1/MigratedAddSub.saved
bag_migration_tests/msg_gen1/MigratedExplicit.saved
bag_migration_tests/msg_gen1/MigratedImplicit.saved
bag_migration_tests/msg_gen1/MigratedMixed.saved
bag_migration_tests/msg_gen1/PartiallyMigrated.saved
bag_migration_tests/msg_gen1/Renamed1.saved
bag_migration_tests/msg_gen1/Simple.saved
bag_migration_tests/msg_gen1/SimpleMigrated.saved
bag_migration_tests/msg_gen1/SubUnmigrated.saved
bag_migration_tests/msg_gen1/Unmigrated.saved
```

And will create the following *.bmr* migration rules, that the user need to implement and make valid as they are created:

``` bash
bag_migration_tests/msg_gen1/Constants.bmr
bag_migration_tests/msg_gen1/MigratedAddSub.bmr
bag_migration_tests/msg_gen1/MigratedImplicit.bmr
bag_migration_tests/msg_gen1/Renamed1.bmr
bag_migration_tests/msg_gen1/SimpleMigrated.bmr
```
2. Generate migration rules from the generation \#3 to the **current** one:

This assumes we've previously create the rules from generation \#1 to \#2 and \#2 to \#3. With those rules created (from scratch), now we can create the ones from \#3 to the **current** generation with:

``` bash
$ rosrun test_rosbag generate_rules gen3 current
```

The output and process will be similar to the previous example.

At the end, the user should organize all the rules from different generations. It's recommended to put all the rules for the same message in a single file. This file would have all the migration rules for it, for all the generations that need them.
