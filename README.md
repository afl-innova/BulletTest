# BulletTest
Some code to test bullet physics collision detection.

## How to build:

**Windows**
```sh
cmake -DBULLET_ROOT="${BULLET_ROOT}" -G "Visual Studio 16 2019" -B build
cmake --build build
```

**Linux/Mac OS:**
```sh
cmake -DBULLET_ROOT="${BULLET_ROOT}" -G "Unix Makefiles" -B build
cmake --build build
```
