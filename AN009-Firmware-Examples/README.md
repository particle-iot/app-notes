# AN009 - Firmware Examples

This application note contains a number of examples of application firmware for common use cases.

## Publishing Examples, Sleeping Cellular Devices

There are three examples of periodically publishing data from an E Series, Electron, Boron, or B Series SoM that mostly sleeps.

#### Wake Publish Sleep Cellular

This example illustrates:

- Checking that the battery has sufficient capacity to connect.
- Sleep on multiple connection failure to preserve battery.
- Hour sleep, uses hibernation (`SLEEP_MODE_DEEP`).
- This example is best for long sleep times, 15 minutes or more.
- Use SEMI_AUTOMATIC mode.
- Stay awake for firmware updates (optional).

#### Stop Sleep Cellular

This example illustrates:

- Checking that the battery has sufficient capacity to connect.
- Sleep on multiple connection failure to preserve battery.
- 5 minute sleep, using stop mode sleep with cellular standby (`SLEEP_NETWORK_STANDBY`).
- This example is best with short sleep times, under 15 minutes or if you want to be able to publish very quickly after waking up.
- Customizable sending of device diagnostics.
- Customizable software update behavior.

#### EEPROM Samples

This is the most complicated example. It illustrates:

- 5 minute sleep, using stop mode sleep with cellular standby (`SLEEP_NETWORK_STANDBY`).
- This example is best with short sleep times, under 15 minutes or if you want to be able to publish very quickly after waking up.
- Customizable sending of device diagnostics.
- Customizable software update behavior.
- Stores samples in EEPROM so they are preserved even if the connection fails.
- Periodic sampling with minimal drift, even with connection failures.
