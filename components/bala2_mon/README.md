# bala2_mon

Watch and tune a BALA2 Fire firmware from a computer or a phone, while it runs. It works over the link of [bala2_link](../bala2_link), and it is for the BALA2 Fire only (ESP32).

It gives a firmware two things:

- **Variables.** The firmware names the numbers it wants to show and change. After that, a command sets one, `?` lists them, and the phone page shows a box for each.
- **Values that stay.** `mon_save()` writes the variables to flash (NVS) and `mon_load()` reads them at the next start, so a good set needs no new firmware.
- **Fast samples.** The control loop hands in one sample per tick. They go out over UDP as they come, about 20 ms later.

[06-balance](../../examples/bala2-fire/06-balance) uses it.

## Why it is built this way

The balance loop itself stays on the robot. A robot of this size falls in about a tenth of a second, and Wi-Fi adds 10 to 15 ms each way, with late packets now and then. One lost packet would be a fall.

But everything around the loop can be as fast as the loop:

- A command writes the number at once, and the control loop uses it on its next tick, 5 ms later at most.
- The loop pays almost nothing. A variable is a plain number that it reads as before. A sample is a copy of a few floats into a ring, with no lock and no wait.
- All the rest (text, sockets, the table) runs in the tasks of core 0.

So a computer can close an outer loop over Wi-Fi, at 20 to 50 Hz: read the fast samples, and move a setpoint like the balance point or a speed.

## Use it

1. Require it in `main/CMakeLists.txt` with `PRIV_REQUIRES bala2_mon` (and set up `bala2_link` as its README says).

2. Name the variables at boot, before `link_start()`. Each one has the range a command may set it to:

   ```c
   #include "bala2_mon.h"

   static volatile float kp = 3.5f;
   static volatile int servo_us = 1500;

   mon_float("kp", &kp, 0, 20);
   mon_int("servo", &servo_us, 500, 2500);
   mon_rec_columns("tilt,rate,u");
   ```

   A name has up to 7 characters. There is room for 32 variables.

3. Pass the command lines on. `mon_command()` returns false if the line is not for it:

   ```c
   static void handle_command(const char *line)
   {
       if (!mon_command(line)) {
           mon_say("unknown command: %s\n", line);
       }
   }
   ```

4. In the control loop, on every tick:

   ```c
   const float sample[MON_REC_COLS] = {tilt, rate, u};
   mon_rec_add(time_ms, sample);
   ```

   Call it on every tick, also when nothing is being taken: a take starts and stops inside it. With nothing asked for, it returns at once.

5. In a task on core 0, 50 times a second or more:

   ```c
   mon_rec_send();
   ```

6. To keep values over a restart, call `mon_load()` once after `link_start()`, and give the firmware two commands that call `mon_save()` and `mon_defaults()`. A flash write stops both cores for a few ms, so the firmware must refuse them while the robot balances. `mon_not_saved("servo")` leaves a variable out. Each value has its own entry in the NVS space `mon`, under its name, so a firmware with other variables still reads the ones it knows. `mon_defaults()` deletes them all and puts back the values the firmware started with. `mon_stored()` tells how many values are in flash.

7. For the phone page, put `mon_json()` in the state the firmware gives to `bala2_link`.

## Over UDP

| You send | The robot does |
| --- | --- |
| `kp 3.7` | Sets `kp`, kept inside its range, and answers with the whole list |
| `?` | Answers with the list: `gains kp 3.5 ki 10 ...` |

The firmware starts a take of fast samples with `mon_rec_start(n)`. In 06-balance that is the command `rec 3` (3 seconds). The answer looks like this:

```
rec_start 600 samples
rec,t_ms,tilt,rate,acc_tilt,x,v,u
rec,48210,7.61,-1.2,7.58,0.0012,0.004,31
...
rec_end lost 0
```

`lost` counts samples that found the ring full. The ring holds 1024 samples, 5 seconds at 200 Hz, so samples are only lost when the link stops for that long.

## What we measured on the robot (2026-11-12)

On the home network, with the robot lying still:

- A command and its answer: 13 ms (median of 28), 86 ms at worst.
- Fast samples: 600 of 600 arrived in a 3 s take, 5 ms apart, none lost. A sample arrived 18 ms after the fastest one (median), 55 ms for 95% of them, 74 ms at worst.
- A value outside its range was kept inside it: `kp 999` gave 20, `kp -5` gave 0.

The same while the robot balanced, with the motors at work:

- A command and its answer: 9 ms (median of 60), 18 ms for 95% of them, 36 ms at worst. None went unanswered.
- Fast samples: four takes of 10 s, 2000 of 2000 samples each, none lost. A sample arrived 17 to 19 ms after the fastest one (median), 54 to 57 ms for 95% of them, 74 to 103 ms at worst.

So the link is as fast while the robot balances as when it lies still. Not measured: the same on the robot's own network.
