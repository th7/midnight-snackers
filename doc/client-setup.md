# Setting up a client

How a teammate's own computer goes from nothing to running code in the
simulator and deploying it to the robot. Clients are Windows machines; the
steps are the same on a Mac, with the paths given for both.

Only the host computer reaches the robot. A client deploys through the
**robot relay** on the host, which passes the client's adb connection on to
the hub once the admin has approved the client's login.

## 1. Install Android Studio

Install [Android Studio](https://developer.android.com/studio) (Narwhal 3
Feature Drop or later, as the FTC SDK asks for) with its default
components. The first start downloads the Android SDK, which holds `adb`.

## 2. Clone the project

Clone `https://github.com/th7/midnight-snackers` with your own GitHub
account, then open the folder in Android Studio (**File › Open**) and let
the Gradle sync finish. The first sync downloads the build's dependencies
and takes a few minutes.

## 3. A first run in the simulator

Pick the **Simulator** run configuration in the toolbar and press Run. The
Run window prints a URL, `http://127.0.0.1:21988/`; open it in a browser.
The Simulate tab lists Reginald's op modes. Pick one and press Run.

Edit the code in Android Studio and save; the next Run in the page runs what
you saved. A compile error is shown in the page with its file and line. The
simulator answers this computer alone, so nobody else on the network sees it.

From a terminal the same is `gradlew.bat :TeamCode:simulator` on Windows and
`./gradlew :TeamCode:simulator` on a Mac.

## 4. Log in to the host

Open the host's address in a browser, `http://<host>:21986/`, where `<host>`
is the host computer's LAN address (the admin's page lists it). Enter your
name and wait while the admin approves you.

Once approved, the page's header gives the line to run, such as

```
adb connect 192.168.1.20
```

The relay admits connections from the address your login came from, so log
in from the computer you deploy from. If your address changes, log in again.

## 5. The detailed field

The project commits only the low field model. The host builds the medium and
high ones from the field's CAD, with a key only the host holds, and offers
them to every computer with an approved login. Tell the simulator where the
host is with the `ROBOT_GATEWAY` environment variable, set to the same address
as in step 4: once on Windows, in a terminal,

```
setx ROBOT_GATEWAY 192.168.1.20
```

and on a Mac `launchctl setenv ROBOT_GATEWAY 192.168.1.20`, which lasts until
the Mac restarts. Then quit and reopen Android Studio, so that it and the
simulator it starts see the setting. Add `:<port>` only if the admin moved the host's page off
port 21986.

Each time the simulator starts with the host in reach, it fetches the models
the host built that it does not already hold, and the Run window says what
it fetched. It keeps them on this computer:

| | Where the field models are kept |
|---|---|
| Windows | `%LOCALAPPDATA%\midnight-snackers\field-models` |
| Mac | `~/Library/Application Support/midnight-snackers/field-models` |

and draws the most detailed one it holds, away from the host too. After the
admin rebuilds the models, the next start on the LAN fetches the new ones. A
fetch cut off partway leaves the model fetched before. The line under the
field in the page says which model it draws. With nothing kept, that is the
low field committed with the project.

## 6. Connect adb to the robot through the host

Android Studio keeps `adb` in its SDK, not on the `PATH`:

| | Where `adb` is |
|---|---|
| Windows | `%LOCALAPPDATA%\Android\Sdk\platform-tools\adb.exe` |
| Mac | `~/Library/Android/sdk/platform-tools/adb` |

**Settings › Languages & Frameworks › Android SDK** shows the SDK's location
if it was put somewhere else. Use this `adb` and no other: two versions of adb
on one computer stop each other's servers.

In Android Studio's own **Terminal** on Windows:

```
& "$env:LOCALAPPDATA\Android\Sdk\platform-tools\adb.exe" connect 192.168.1.20
& "$env:LOCALAPPDATA\Android\Sdk\platform-tools\adb.exe" devices
```

and on a Mac:

```
~/Library/Android/sdk/platform-tools/adb connect 192.168.1.20
~/Library/Android/sdk/platform-tools/adb devices
```

with the line from the header in place of `192.168.1.20`. `adb devices` lists
the robot as `192.168.1.20:5555  device`, named by the host's address.

## 7. A first deploy

Pick the **TeamCode** run configuration and the robot in the device list
beside it, and press Run. Android Studio builds the app, installs it on the
robot and starts it; Logcat shows the robot's output, and the debugger
attaches as it would to a phone on a cable.

## When it does not connect

- **`adb connect` says the connection was refused, or the device is listed
  as `offline` and goes away.** The relay closes a connection from an
  address with no approved login. Check the host's page still says you are
  approved, and that you logged in from this computer. The host's terminal
  prints each refused address.
- **`adb connect` hangs, then fails.** The host could not reach the hub. The
  admin page's **Robot relay** panel says whether the hub was reachable when
  last tried, and why not. The host must be on the robot's Wi-Fi as well as
  the LAN.
- **The admin revoked the login.** Every connection relayed from that
  address closes, unless another approved login came from the same address.
  Log in again and ask for approval.

- **The simulator draws the low field while the host has the detailed
  ones.** The simulator's Run window says why: `ROBOT_GATEWAY` is not set,
  the host could not be reached, or the host refused this computer because
  no approved login came from its address. Log in from this computer again,
  and restart the simulator once the admin approves you.

## For the admin

The host listens for the relay on port 5555, adb's own, so the line needs no
port. `ROBOT_RELAY_PORT` sets another; the line in each teammate's header
then names it. The hub is reached at `192.168.43.1:5555`, the Control Hub's
address on its own Wi-Fi; `ROBOT_HUB_ADDRESS` (as `host:port`) sets another.
The first time the host listens, macOS may ask whether Java may accept
incoming connections: allow it, or no client reaches the relay.

The admin page lists, per user, whether a connection relayed for them is
open, and the hub's state.
