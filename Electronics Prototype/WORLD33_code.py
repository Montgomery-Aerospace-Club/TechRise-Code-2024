from worldview_lib import *
import time
import sdcardio
import board
import busio
import digitalio
import storage

#LED:
#When the PBF header is inserted and data is not streaming = LED blinks with interval of 5 seconds
#When the PBF header is removed and data is not streaming = One second ON, One second OFF
#When the PBF header is inserted and data is streaming = LED blinks with interval of 3 seconds
#When the PBF header is removed and data is streaming = LED ON, does not blink


# Use any pin that is not taken by SPI
SD_CS = board.D10

# Connect to the card and mount the filesystem.
spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
sdcard = sdcardio.SDCard(spi, SD_CS)
vfs = storage.VfsFat(sdcard)
storage.mount(vfs, "/sd")

def main():
    sim = Simulator()

    while True:
        sim.update()

        if sim.new_data:
            current_status = sim.status
            status_change_message = ""

            # Check if the status has changed
            if current_status != sim.prev_status:
                # Create the status change message
                if current_status == STATUS_INITIALIZING:
                    status_change_message = "Balloon is initializing\n"
                elif current_status == STATUS_LAUNCHING:
                    status_change_message = "Balloon is launching\n"
                elif current_status == STATUS_FLOATING:
                    status_change_message = "Balloon is floating\n"
                elif current_status == STATUS_TERMINATING:
                    status_change_message = "Balloon is terminating\n"
                # Update the previous status
                sim.prev_status = current_status

            print("New data received:")
            print("Time:", sim.time)
            print("Latitude:", sim.latitude)
            print("Longitude:", sim.longitude)
            print("Altitude:", sim.altitude)
            print("Speed:", sim.speed)
            print("Heading:", sim.heading)
            print("Velocity Down:", sim.velocity_down)
            print("Pressure:", sim.pressure)
            print("Temperature:", sim.temperature)
            print("Flight Status:", sim.status)
            print("PBF State:", "Removed" if sim.pbf else "Inserted")
            print("GO LED State:", "Lit" if sim.go else "Off")
            print("-----------------------------------")

            # Write data to the SD card
            with open("/sd/WORLD.txt", "a") as f:
                if status_change_message:
                    f.write(status_change_message)
                # In data_to_save make sure there is ONLY the data you want to save to the SD Card
                data_to_save = ("{},{},{},{},{},{},{},{},{}\n".format(sim.time,
                                                                      sim.latitude,
                                                                      sim.longitude,
                                                                      sim.altitude,
                                                                      sim.speed,
                                                                      sim.heading,
                                                                      sim.velocity_down,
                                                                      sim.pressure,
                                                                      sim.temperature))
                f.write(data_to_save)
                print("Saving to SD card:", data_to_save)

        # Sleep for a short duration to avoid overwhelming the console with data
        time.sleep(1)

if __name__ == "__main__":
    main()
