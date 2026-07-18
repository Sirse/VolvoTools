# VolvoTools tests (gtest)

Юнит-тесты на весь проект. Сейчас покрыты:
- **CommonProtocolTests** — чистые функции `Common`: hex-парсинг, форматирование/декод DTC,
  парсеры ответов `19 02` / `19 04`-`06`, DID (`22`).
- **CliParserTests** — `parseOptions`: маппинг команд, дефолты, именованные значения
  (`--type`, `--sub`, `--session`), валидация-отказы, группы `did`/`can`.

Тесты не требуют железа и логов. End-to-end `run*` (обмен с ECU) сюда не входят — для них
нужен мок `J2534Channel` или записанные ответы.

## Сборка и запуск

Требуется gtest из Conan (добавлен в `conanfile.txt`). Из-за того что часть зависимостей
кэширована под `cppstd=17`, а старые рецепты не configure-ятся на CMake 4, ставить так:

```sh
export CMAKE_POLICY_VERSION_MINIMUM=3.5        # для сборки старых рецептов из исходников
conan install . -s build_type=Release -s compiler.cppstd=17 --build=missing
cmake --preset conan-default
cmake --build build --config Release --target VolvoToolsTests
ctest --test-dir build -C Release               # или запустить exe напрямую
```

Отключить тесты в основной сборке: `-DVOLVOTOOLS_BUILD_TESTS=OFF`.

## Обе разрядности (x86 + x64), как в CI

CI (`.github/workflows/build.yml`) всегда собирает **и x86, и x64** — J2534-драйвер
грузится в рантайме через `LoadLibrary`, поэтому инструмент работает только с драйвером
своей разрядности. Локально то же самое одной командой:

```powershell
.\build_all.ps1                 # x86 -> build-x86\, x64 -> build-x64\ (Release)
.\build_all.ps1 -Config Debug
.\build_all.ps1 -Arch x64       # только одна разрядность
```

Деревья раздельные (`build-x86\` / `build-x64\`), чтобы не затирать друг друга.
Запуск тестов нужной разрядности:

```powershell
ctest --test-dir build-x64 -C Release
ctest --test-dir build-x86 -C Release
```

## Архитектура

Логика VolvoDiag собирается в статическую `VolvoDiagCore`, exe `VolvoDiag` — тонкий `main`.
Тест-таргет линкуется с `VolvoDiagCore` (и через неё с `Common`). Для нового приложения
повторить тот же паттерн (core-lib + thin main), чтобы его логика была тестируемой.
