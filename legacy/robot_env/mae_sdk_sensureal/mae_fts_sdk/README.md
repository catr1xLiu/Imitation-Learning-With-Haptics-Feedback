# MAE Force-Torque Sensor (FTS) Python Software Development Kit (SDK) Package

## Requirements

Prior to installing the `mae_fts_sdk` package, the `mae_sdk` package must be
installed. To install the `mae_sdk` package, open the Terminal and execute the
following steps:

1. `cd` into the folder that contains the `mae-sdk` folder.
2. `pip install ./mae-sdk`

## Installation

To install the `mae_fts_sdk` package, open the Terminal and execute the
following steps:

1. `cd` into the folder that contains the `mae-fts-sdk` folder.
2. `pip install ./mae-fts-sdk`

## Sample Code for using MAE FTS Python SDK

The `mae_fts_sdk_sample` has two main purposes:
- To show how the `mae_fts_sdk` can be used to quickly prototype sensor
  communication;
- To support the `mae_fts_sdk` package in serving as a documented example on how
  to communicate and operate the MAE Force-Torque Sensor. 

In the `mae_fts_sdk_sample` script, there are three modes that takes FTS API
commands as arguments:

- `send_once`
- `send_loop`
- `stream`

The command below start streaming FTS data.

```
./mae_fts_sdk_sample stream FtsCommand.STREAM_FT_START
```

> _The `mae_fts_sdk` package supports sensor devices that communicates either
> via Serial or UDP. Check the manual of your MAE Force-Torque Sensor for the
> available communication interfaces in your device._

### FTS Commands

Check the [fts_commands.py](src/mae_fts_sdk/fts_commands.py) for the list of
commands and their hex values that are available for the MAE FTS.

`<TAB>` can also be used to trigger autocomplete for the `mae_fts_sdk_sample`
script. This is highly recommended to facilitate its usage.


### Enabling Auto Complete

The `mae_fts_sdk_sample` script supports auto-complete. In order to enable it
you must run the following command in your shell once.

```
./enable_auto_complete
```

This will add the following line to the `.bashrc` and/or the `.zsh`
configuration files:

```
eval "$(register-python-argcomplete ./mae_fts_sdk_sample)"
```

## Sensor Installation Procedure

1. **Allow Mechanical Stress to Settle**  
   After mounting the sensor, wait for internal stresses to stabilize.  
   - The required wait time depends on factors such as installation stress, surface flatness, and the materials in contact with the sensor.

2. **(Optional) Wait for Thermal Equilibrium**  
   If nearby actuators generate heat that may affect the sensor, we strongly recommend waiting **30 to 60 minutes** to allow heat conduction and temperature gradients to stabilize.

3. **Zero the Sensor**  
   Ensure no external loads are applied to the sensor, then:

   1. Run:
      ```bash
      mae_fts_sdk_sample send_once FtsCommand.TRANSDUCER_SET
      ```
      This sets the transducer's baseline to match the current mechanical state.

   2. Run:
      ```bash
      mae_fts_sdk_sample send_once FtsCommand.BIAS_SET
      ```
      This tares the force-torque output to remove any offsets.

4. **Save Settings**  
   Persist the new configuration by running:
   ```bash
   mae_fts_sdk_sample send_once FtsCommand.SETTINGS_SAVE

### (Optional) Changing Sensor IP Address

By default, the UDP communication of the sensor uses the IP Address
`192.168.1.11`, and the port `10547`. The IP Address can be reconfigured with
the following steps:

1. **Set new IP Address**  
   Run:
   ```base
   mae_fts_sdk_sample send_once FtsCommand.IP_ADDRESS_SET 192.168.1.12
   ```
   This sets the new IP Address to `192.168.1.12`.
   
2. **Save Settings**  
   Persist the new configuration by running:
   ```bash
   mae_fts_sdk_sample send_once FtsCommand.SETTINGS_SAVE --ip 192.168.1.12
   ```
   Note: When using a custom IP address, you must specify it using the `--ip`
   flag in all `mae_fts_sdk_sample` commands.


## Streaming Force-Torque values
   
1. **Start FT Stream**     
   Run:
   ```bash
   mae_fts_sdk_sample stream FtsCommand.STREAM_FT_START
   ```

2. **Stop FT Stream**     
   Press `CTRL+C` to interrupt the `mae_fts_sdk_sample` will automatically
   issue a `FtsCommand.STREAM_STOP` command to stop stream. 

3. **(Optional) Save Settings**  
   Save Settings to persist any changes on the peak values read by the sensor
   into memory:
   ```bash
   mae_fts_sdk_sample send_once FtsCommand.SETTINGS_SAVE
   ```

<div style="text-align: center;">
<i>MAE Robotics - 2025-08-05</i>
</div>

