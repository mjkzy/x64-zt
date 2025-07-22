# x64 ZoneTool
A fastfile unlinker and linker for various x64 Call of Duty titles. 

Maps for supported games assets can be built & dumped by referring to the [wiki](https://github.com/Joelrau/x64-zt/wiki).

## Supported Games
* **IW6** (*Call of Duty: Ghosts*)
* **S1** (*Call of Duty: Advanced Warfare*)
* **T7** (*Call of Duty: Black Ops 3*) ***[dumping only]***
* **H1** (*Call of Duty: Modern Warfare Remastered*)
* **H2** (*Call of Duty: Modern Warfare 2 Campaign Remastered*)
* **IW7** (*Call of Duty: Infinite Warfare*) ***[no custom maps]***

## Wiki
Check out the [Wiki](https://github.com/Joelrau/x64-zt/wiki) for useful guides & information on how to port maps and use zonetool.

## Commands
* `loadzone <zone>`: Loads a zone
* `unloadzones`: Unloads zones
* `verifyzone <zone>`: Lists assets in a zone
* `dumpzone <zone>`: Dumps a zone
* `dumpzone <target game> <zone> <asset filter>`: Dumps a zone converting assets for a specific game
* `dumpasset <type> <name>`: Dumps a single assset
* `dumpmap <map>`: Dumps all required assets for a map
* `dumpmap <target game> <map> <asset filter> <skip common>`: Dumps and converts all required assets for a map

  ### Definitions
  * `asset filter`: A filter specifying all the asset types that should be dumped, if not specified or empty it will dump all asset types.
  Asset types are separated by **commas**, **`_`** indicates and empty filter.   
    * Example: `dumpzone h1 mp_clowntown3 sound,material,techset,rawfile`
    * Example: `dumpmap h1 mp_clowntown3 _ true`
  * `skip common`: Skips common zones when dumping a map, can be `true` or `false`.
  * `target game`: The game to convert the assets to.

## Conversion support
The conversions for how assets can translate is showed on a table below:

✔️ – Fully supported
⚠️ – Partial / Limited
❌ – Not supported

|            | **IW6** | **S1** | **H1** | **H2** | **T7** | **IW7** |
| ---------- | ------ | ------ | ------ | ------ | ------ | ------ |
| **IW6**    | ✔️     | ❌     | ✔️     | ✔️     | ❌     | ✔️     |
| **S1**     | ❌     | ✔️     | ✔️     | ✔️     | ❌     | ❌     |
| **H1**     | ❌     | ⚠️     | ✔️     | ✔️     | ❌     | ⚠️     |
| **H2**     | ❌     | ❌     | ✔️     | ✔️     | ❌     | ❌     |
| **T7**     | ❌     | ❌     | ⚠️     | ❌     | ❌ | ⚠️     |
| **IW7**    | ❌     | ❌     | ✔️     | ❌     | ❌     | ✔️ |  
