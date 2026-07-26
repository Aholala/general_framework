# General Framework Engineering Rules

These rules apply to the complete repository. They are mandatory for future
Algorithm, App, BSP and Module work unless the user explicitly overrides them.

## Layer and naming rules

- Algorithm files, types, functions and variables use the `alg_` prefix.
- App files, types, functions and variables use the `app_` prefix.
- BSP files, types, functions and variables use the `bsp_` prefix.
- Module files, types, functions and variables use the `module_` prefix.
- All identifiers except macros and enumeration constants use lower-case
  `snake_case`. CamelCase and PascalCase are forbidden.
- Structure, enumeration, callback and operation-table typedef names use the
  same lower-case prefix and end in `_t`, for example `bsp_usart_t`,
  `module_bmi088_config_t` and `alg_pid_status_t`.
- Public functions use `layer_module_action`, for example
  `alg_pid_update`, `bsp_usart_transmit` and `module_bmi088_read`.
- Macros and enumeration constants remain upper-case `SNAKE_CASE` and include
  the complete layer and module prefix.
- Use complete English words. Avoid ambiguous abbreviations and single-letter
  variables except conventional loop indexes or mathematical notation.
- Every module lives in one flat module directory. Do not create `Inc` and
  `Src` subdirectories.
- Public declarations belong in the module header. Private helpers, private
  state, private vtables and private constants must be `static` in the `.c`
  file whenever possible.
- Immutable configuration, operation tables and input-only pointers must be
  `const`.
- Current-object parameters are always named `me`. In private dispatch helpers,
  use a type-specific base name such as `timer_base`, `spi_base` or
  `device_base`; use `super` only for the embedded parent member.
- Names must express units and meaning: `timeout_ms`, `frequency_hz`,
  `period_ticks`, `reference_voltage_v`, `sample_count`, `transmit_data`,
  `receive_data`, `user_context`, `filter_config`, and `receive_fifo`.
- Avoid context-free names such as `ctx`, `len`, `val`, `tmp`, `buf`, `tx`,
  `rx`, `ptr`, `obj`, `base`, `level`, or `data` when a more precise name is
  available. Mathematical local variables are exempt where conventional.
- Every `if`, `else`, `for`, `while`, and `do` body uses braces, including a
  one-statement body. Do not compress functions or control flow onto one line.
- Group each `.c` file in this order: private conversion/validation helpers,
  private virtual implementations, `static const` vtable, constructor, public
  interface functions, notification/callback entry points.

## Mandatory C object-oriented model

- Use C11 only. Do not require C++.
- Every polymorphic BSP object contains a base struct named `super` as its first
  member.
- Every base object contains a virtual-table pointer named `vptr`.
- Operation tables are immutable `static const` objects.
- Public virtual interfaces receive the base-class pointer as the first
  parameter, named `me`.
- Public code calls non-virtual interface functions such as
  `bsp_usart_transmit(me, ...)`; those functions validate the object and dispatch
  through `me->super.vptr`.
- Derived virtual implementations recover the complete derived object with
  `BSP_CONTAINER_OF(me, derived_type, super)`. Do not use global peripheral
  handles to find an object.
- Operation-table inheritance uses struct nesting. A derived/interface vtable
  embeds its parent operation table as the first member named `super`.
- Object inheritance uses struct nesting. Do not emulate inheritance by casting
  unrelated structures.
- A base-class pointer must be usable for multiple different derived devices,
  providing the same public operation with different behavior.
- Constructors are named `layer_module_init`, such as `bsp_usart_init` and
  `module_bmi088_init`. They receive object storage and a
  `const` configuration object, initialize every field, and leave the object in
  a deterministic state on failure.
- Destruction/deinitialization is virtual through `bsp_device_deinit` when the
  object owns an initialized device binding.
- Never return initialized polymorphic objects by value and never copy them by
  value after initialization. Store and pass pointers.

## BSP portability and hardware injection

- Generic BSP headers and sources must not include vendor headers such as
  `stm32h7xx_hal.h`.
- Generic BSP code must not mention a concrete MCU family, peripheral instance,
  register, IRQ name or vendor HAL function.
- Hardware access is injected with two fields stored in the derived device:
  an opaque `device_handle` and a `const driver_ops` table.
- A low-level port implements only the driver operation table once per platform.
  It must not reimplement the BSP object model or public API.
- All board instance selection, pins, channels, handles, IRQ mappings and
  logical device names belong in `User/Bsp/board_config.h`.
- `board_config.h` assembles instances; it must not contain business logic.
- App and Module code depend only on base BSP interfaces and receive base-class
  pointers through configuration or constructors.
- No BSP object may depend on CubeMX-generated global names such as `huart1`,
  `hspi1` or `hadc1` internally.

## Multi-instance and callbacks

- All runtime state belongs to the object instance. Do not use mutable
  file-scope singleton state for a peripheral.
- Multiple objects using the same class must operate independently.
- Callbacks store both a callback function and an opaque user context in the
  owning object.
- IRQ/HAL callback routing belongs to the injected port or board assembly. The
  generic BSP library exposes explicit `notify` entry points where required.
- ISR callbacks must remain non-blocking.

## Error handling and memory

- Use a shared `bsp_status_t` for BSP errors.
- Validate `me`, initialization state, pointers, sizes, ranges and unsupported
  operations before virtual dispatch.
- An absent optional virtual operation returns `BSP_STATUS_UNSUPPORTED`.
- Do not allocate dynamic memory. Object storage, buffers and workspaces are
  caller-owned or static.
- Avoid hidden copies of large buffers. Document aliasing and lifetime rules.

## Verification

- Do not create repository-local `Test`, `test` or `tests` directories unless
  the user explicitly requests tests again.
- Document recommended validation cases in each module README instead of
  shipping test sources inside the framework.
- When an external validation harness is available, cover constructor failures,
  uninitialized objects, boundary values, optional operations, callbacks and
  at least two fake derived behaviors for polymorphic interfaces.
- Cross-compile the full target project after integration.
- Keep generated validation caches and probe objects out of source module
  folders.
