# Tests Plan

Этот файл не содержит реализацию тестов. Это список того, какие тесты стоит написать, если переносить и структурировать проверки из текущего корня `src/tests/` в более явную схему.

Ниже для каждого будущего теста указано:
- что проверяется;
- на какие существующие тесты в `src/tests/` он опирается.

## Unit

### `unit/test_serial_line_search.py`

- `test_search_returns_first_matching_device`
  Проверяет инвариант: поиск по regex возвращает путь к подходящему устройству.
  Основа: `test_serial_line_search_match`.

- `test_search_raises_when_match_not_found`
  Проверяет инвариант: отсутствие совпадения приводит к `SerialLineSearchNotFound`.
  Основа: `test_serial_line_search_not_found`.

- `test_search_rejects_invalid_regex`
  Проверяет инвариант: сломанный regex приводит к `SerialLineSearchInvalidPattern`.
  Основа: `test_serial_line_search_invalid_pattern`.

- `test_search_raises_for_unreadable_directory`
  Проверяет инвариант: несуществующая директория приводит к `SerialLineSearchDirectoryError`.
  Основа: `test_serial_line_search_directory_error`.

- `test_search_requires_non_empty_pattern`
  Проверяет инвариант: пустой pattern запрещён.
  Основа: `test_serial_line_search_requires_pattern`.

### `unit/test_sky_physics_ha.py`

- `test_stellar_speed_constant_matches_expected_value`
  Проверяет инвариант: расчёт звездной скорости не съехал.
  Основа: `test_stellar_speed`.

- `test_ha_from_string_parses_valid_values`
  Проверяет инвариант: корректный `HH:MM:SS` стабильно парсится.
  Основа: `test_ha_from_string_valid`, `test_hours_from_string_valid`.

- `test_ha_from_string_rejects_invalid_format`
  Проверяет инвариант: некорректные строки HA отвергаются.
  Основа: `test_ha_from_string_invalid_format`, `test_hours_from_string_invalid_format`.

- `test_ha_rounds_components_consistently`
  Проверяет инвариант: строковое представление RA корректно округляет граничные значения.
  Основа: `test_ha_rounding_for_components`, `test_hours_rounding_for_components`.

- `test_ha_wrap_normalizes_values_outside_single_circle`
  Проверяет инвариант: значения вне 24h нормализуются в эквивалентный диапазон.
  Основа: `test_ha_wraps_values_outside_single_circle`.

- `test_ha_moving_wrap_chooses_shortest_signed_delta`
  Проверяет инвариант: `moving_wrap()` всегда оставляет кратчайший signed delta.
  Основа: `test_ha_moving_wrap_returns_shortest_signed_delta`.

- `test_ha_negation_preserves_type`
  Проверяет инвариант: unary minus возвращает `Ha`, а не число.
  Основа: `test_ha_negation_returns_same_type_with_inverted_raw`.

- `test_ha_division_and_multiplication_by_second_roundtrip`
  Проверяет инвариант: `Ha / Second -> HaPerSecond`, затем обратно даёт исходное значение.
  Основа: `test_ha_divide_and_multiply_by_seconds_roundtrip`.

- `test_ha_division_rejects_unsupported_types`
  Проверяет инвариант: неподдерживаемые типы не приводятся молча.
  Основа: `test_ha_division_rejects_unsupported_type`.

### `unit/test_sky_physics_dec.py`

- `test_dec_from_string_parses_valid_values`
  Проверяет инвариант: корректный DEC-формат стабильно парсится.
  Основа: `test_dec_from_string_valid`, `test_dec_from_string_valid` из `test_ra_dec_containers.py`.

- `test_dec_from_string_rejects_invalid_format`
  Проверяет инвариант: неверный DEC-формат отвергается.
  Основа: `test_dec_from_string_invalid_format` в обоих файлах.

- `test_dec_rounds_components_consistently`
  Проверяет инвариант: строковое представление DEC корректно округляет пограничные значения.
  Основа: `test_dec_rounding_for_components`, `test_dec_from_arcseconds`.

- `test_dec_wrap_normalizes_values_outside_single_quarter_circle`
  Проверяет инвариант: значения вне физического диапазона DEC правильно отражаются обратно.
  Основа: `test_dec_wraps_values_outside_single_quarter_circle`.

- `test_dec_negation_preserves_type`
  Проверяет инвариант: unary minus возвращает `Dec`.
  Основа: `test_dec_negation_returns_same_type_with_inverted_raw`.

- `test_dec_division_and_multiplication_by_second_roundtrip`
  Проверяет инвариант: `Dec / Second -> DecPerSecond`, затем обратно даёт исходное значение.
  Основа: `test_dec_divide_and_multiply_by_seconds_roundtrip`.

- `test_dec_to_degrees_matches_expected_sign`
  Проверяет инвариант: знак не теряется при конвертации в градусы.
  Основа: `test_dec_to_degrees_negative`, `test_dec_from_degrees_valid`.

- `test_dec_roundtrip_random_values`
  Проверяет инвариант: случайные DEC значения переживают строковой roundtrip.
  Основа: `test_dec_roundtrip_random_degrees`.

### `unit/test_sky_physics_model.py`

- `test_arithmetic_operations_return_expected_types_and_values`
  Проверяет инвариант: базовая арифметическая модель `Second/Ha/Dec/...` типобезопасна.
  Основа: `test_sky_physics_test_model`.

- `test_comparisons_work_for_equal_and_ordered_values`
  Проверяет инвариант: сравнения между однотипными контейнерами корректны.
  Основа: `test_ha_comparisons`, `test_dec_comparisons`, `test_comparisons_on_equal_values`.

- `test_comparisons_use_normalized_values`
  Проверяет инвариант: сравнение происходит по нормализованным значениям, а не по сырому вводу.
  Основа: `test_comparisons_use_normalized_values`.

- `test_comparisons_reject_unsupported_types`
  Проверяет инвариант: сравнение с чужими типами не даёт ложных результатов.
  Основа: `test_comparisons_reject_unsupported_type`.

### `unit/test_guide_compensator_math.py`

- `test_polar_offset_inverse_problem_roundtrip`
  Проверяет инвариант: `compute_guide_speeds()` и `compute_pole_offset()` обратимы в тестовых сценариях.
  Основа: `test_simulated_polar_missaligment`.

### `unit/test_skywatcher_revu24.py`

- `test_revu24_from_int_encodes_mount_format`
  Проверяет инвариант: число кодируется в mount-представление без потери значения.
  Основа: `test_revu24_from_int`.

- `test_revu24_from_mount_decodes_mount_format`
  Проверяет инвариант: данные от mount декодируются обратно в число.
  Основа: `test_revu24_from_mount`.

- `test_revu24_roundtrip_random_values`
  Проверяет инвариант: случайные 24-bit значения переживают roundtrip.
  Основа: `test_revu24_roundtrip_random_values`.

- `test_revu24_rejects_invalid_mount_payloads`
  Проверяет инвариант: мусорные hex payloads отвергаются.
  Основа: `test_revu24_from_mount_invalid`.

- `test_revu24_rejects_invalid_int_values`
  Проверяет инвариант: значения вне диапазона 24-bit отвергаются.
  Основа: `test_revu24_from_int_invalid`.

### `unit/test_skywatcher_status.py`

- `test_status_from_bytes_maps_to_motion_command`
  Проверяет инвариант: status bytes однозначно конвертируются в команду motion mode.
  Основа: `test_status_from_bytes_to_command`.

- `test_status_to_command_encodes_backward_highspeed_slew`
  Проверяет инвариант: конкретный флаговый статус кодируется в правильную команду.
  Основа: `test_status_to_command_encodes_slew_highspeed_backward`.

- `test_status_to_command_encodes_forward_lowspeed_goto`
  Проверяет инвариант: конкретный GOTO status кодируется в правильную команду.
  Основа: `test_status_to_command_encodes_goto_lowspeed_forward`.

- `test_motion_status_ignores_running_and_initialized_flags`
  Проверяет инвариант: motion status зависит только от motion bits, а не от runtime flags.
  Основа: `test_motion_status_ignores_runtime_flags`.

### `unit/test_skywatcher_goto_math.py`

- `test_wrap_delta_move_preserves_target_direction`
  Проверяет инвариант: выбор signed delta для goto не меняет физически ожидаемое направление.
  Основа: `test_wrap_delta_move_keeps_direction`.

- `test_lx200_goto_starts_short_move_before_overshoot`
  Проверяет инвариант: LX200 goto не запускает заведомо длинный первый шаг при короткой цели.
  Основа: `test_lx200_goto_starts_short_motor_move_without_immediate_overshoot`.

### `unit/test_skywatcher_set_rate.py`

- `test_set_ra_rate_zero_stops_motor`
  Проверяет инвариант: нулевая скорость приводит к stop, а не к motion reconfigure.
  Основа: `test_set_ra_rate_zero_stops_motor`.

- `test_set_ra_rate_starts_motor_if_axis_is_idle`
  Проверяет инвариант: при обычном slew-режиме и простое ось стартует после set rate.
  Основа: `test_set_ra_rate_starts_motor_when_axis_not_running`.

- `test_set_ra_rate_does_not_start_motor_for_goto_mode`
  Проверяет инвариант: GOTO-подготовка не должна немедленно стартовать обычный run.
  Основа: `test_set_ra_rate_does_not_start_motor_for_goto_mode`.

- `test_set_motion_stops_running_axis_before_reconfigure`
  Проверяет инвариант: смена motion mode на ходу сначала останавливает ось.
  Основа: `test_set_motion_stops_running_axis_before_mode_change`.

### `unit/test_tmc2209_protocol.py`

- `test_response_parses_ok_and_error_lines`
  Проверяет инвариант: строковый ответ протокола корректно раскладывается на `ok/error/values`.
  Основа: `test_response_from_line_ok`, `test_response_from_line_error`.

- `test_response_rejects_invalid_prefix_duplicate_key_and_empty_value`
  Проверяет инвариант: malformed packet отвергается сразу.
  Основа: `test_response_invalid_prefix`, `test_response_duplicate_key`, `test_response_empty_value`.

- `test_status_and_driver_status_parse_known_payloads`
  Проверяет инвариант: известные status payloads разбираются в typed структуры.
  Основа: `test_status_from_response`, `test_driver_status_from_response`.

- `test_parse_helpers_reject_invalid_values`
  Проверяет инвариант: низкоуровневые parse helpers не принимают мусор.
  Основа: `test_parse_helpers_invalid`.

- `test_adapter_requires_newline_terminator`
  Проверяет инвариант: протокол требует `\n` terminator.
  Основа: `test_adapter_requires_newline_terminator`.

- `test_adapter_status_surfaces_command_error`
  Проверяет инвариант: ошибка адаптера не маскируется и пробрасывается наружу.
  Основа: `test_adapter_status_error`.

- `test_get_param_normalizes_parameter_name`
  Проверяет инвариант: имя параметра нормализуется перед отправкой команды.
  Основа: `test_get_param_normalizes_name`.

- `test_set_param_formats_bool_value`
  Проверяет инвариант: bool кодируется в ожидаемый wire-format.
  Основа: `test_set_param_formats_bool`.

- `test_speed_and_acceleration_ranges_are_validated`
  Проверяет инвариант: диапазоны не отдаются устройству без проверки.
  Основа: `test_set_speed_sps_range`, `test_set_acceleration_range`.

- `test_dec_step_conversion_roundtrip_and_invalid_configs`
  Проверяет инвариант: шаги/DEC конвертируются стабильно, а неверный конфиг ловится заранее.
  Основа: `test_steps_from_dec_roundtrip`, `test_steps_from_dec_negative`, `test_steps_from_dec_invalid_microsteps`, `test_steps_from_dec_invalid_steps_per_rev`.

### `unit/test_tmc2209_lx200.py`

- `test_negative_tracking_rate_maps_to_positive_speed_and_north_direction`
  Проверяет инвариант: знак tracking rate разделяется на абсолютную скорость и направление.
  Основа: `test_set_tracking_rate_negative_uses_positive_speed_and_north_direction`.

- `test_positive_tracking_rate_maps_to_positive_speed_and_south_direction`
  Проверяет инвариант: положительный tracking rate использует ту же абсолютную скорость и противоположное направление.
  Основа: `test_set_tracking_rate_positive_uses_positive_speed_and_south_direction`.

### `unit/test_axis_guide_logic.py`

- `test_ra_guide_applies_east_and_west_with_expected_rate_offsets`
  Проверяет инвариант: RA guiding симметрично меняет tracking rate в обе стороны.
  Основа: `test_ra_guide_applies_east_and_west_with_opposite_rates`.

- `test_dec_guide_applies_north_and_south_with_expected_rate_offsets`
  Проверяет инвариант: DEC guiding симметрично меняет tracking rate в обе стороны.
  Основа: `test_dec_guide_applies_north_and_south_with_opposite_rates`.

- `test_set_tracking_rate_updates_current_rate_and_optional_sky_rate`
  Проверяет инвариант: флаг `update_sky_speed` влияет только на нужное состояние.
  Основа: `test_set_tracking_rate_updates_current_and_sky_by_flag`.

- `test_axis_commands_are_processed_without_waiting_full_compensation_interval`
  Проверяет инвариант: queue команд оси не ждёт большой telemetry/compensation tick.
  Основа: `test_axis_commands_apply_immediately_without_waiting_compensate_interval`.

- `test_splitter_ra_guide_does_not_perturb_dec_rate`
  Проверяет инвариант: guide по RA не задевает DEC кроме обязательного текущего состояния.
  Основа: `test_splitter_ra_guide_applies_without_dec_update`.

- `test_splitter_dec_guide_does_not_perturb_ra_rate`
  Проверяет инвариант: guide по DEC не ломает RA tracking.
  Основа: `test_splitter_dec_guide_applies_without_ra_update`.

### `unit/test_web_control.py`

- `test_example_monitor_exposes_expected_structure`
  Проверяет инвариант: demo monitor публикует поля и actions в ожидаемом виде.
  Основа: `test_example_monitor_exposes_structure_and_actions`.

- `test_registry_updates_snapshot_after_field_write`
  Проверяет инвариант: запись в registry реально меняет monitor snapshot.
  Основа: `test_registry_updates_after_field_write`.

## Hardware

### `hw/test_skywatcher_motor_hw.py`

- `test_set_position_roundtrip`
  Проверяет инвариант: реальный SkyWatcher возвращает выставленную HA позицию.
  Основа: `test_skywatcher_hours_roundtrip`.

- `test_goto_reaches_requested_target_for_small_and_large_deltas`
  Проверяет инвариант: mount доезжает до requested target в обе стороны.
  Основа: `test_slew_to_ra_moves_mount`.

- `test_manual_move_is_rejected_while_goto_is_running`
  Проверяет инвариант: во время GOTO нельзя стартовать обычный slew.
  Основа: `test_move_ra_rejects_goto_in_progress`.

- `test_manual_move_preserves_direction_and_rate`
  Проверяет инвариант: `move_ra()` реально двигает mount с правильным знаком и примерно правильной скоростью.
  Основа: `test_move_ra_speed_and_direction`.

- `test_tracking_preserves_direction_and_rate`
  Проверяет инвариант: `start_tracking()` реально двигает mount с ожидаемой скоростью и знаком.
  Основа: `test_start_tracking_speed_and_direction`.

- `test_zero_tracking_rate_stops_motion`
  Проверяет инвариант: нулевая tracking speed реально останавливает mount.
  Основа: `test_start_tracking_zero_stops_motor`.

### `hw/test_tmc2209_motor_hw.py`

- `test_set_position_updates_real_motor_position`
  Проверяет инвариант: реальный TMC2209 обновляет absolute position.
  Основа: `test_hw_set_position`.

- `test_target_move_runs_for_positive_zero_and_negative_delta`
  Проверяет инвариант: target mode работает для обоих направлений и нулевого delta.
  Основа: `test_hw_slew_delta_moves_both_directions`, `test_hw_slew_delta_zero_sets_and_clears_target`.

- `test_free_run_respects_speed_and_direction`
  Проверяет инвариант: free ride реально едет в нужную сторону с ненулевой скоростью.
  Основа: `test_hw_run_speed_and_direction`.

- `test_stop_during_free_run_freezes_position`
  Проверяет инвариант: stop во время run обнуляет скорость и стабилизирует position.
  Основа: `test_hw_stop_during_run`.

- `test_stop_during_target_move_keeps_partial_progress_and_pending_target`
  Проверяет инвариант: остановка target move оставляет partial progress и не делает вид, будто цель достигнута.
  Основа: `test_hw_stop_during_target_move`.

### `hw/test_splitter_stack_hw.py`

- `test_sync_updates_both_mount_coordinates`
  Проверяет инвариант: sync через splitter обновляет и RA, и DEC в общей системе координат.
  Основа: `test_sync_command_updates_mount_coordinates` из `test_x_splitter_hw_v2.py`.

- `test_stack_is_in_tracking_mode_by_default`
  Проверяет инвариант: после старта стек находится в tracking, а не в halt/goto.
  Основа: `test_mount_in_tracking_mode_by_default`.

- `test_goto_moves_to_target_coordinates`
  Проверяет инвариант: goto на полном стеке доезжает до requested RA/DEC.
  Основа: `test_goto_command_moves_mount_to_target_coordinates`.

- `test_halt_from_slew_returns_to_tracking`
  Проверяет инвариант: halt после ручного slew возвращает систему к tracking.
  Основа: `test_halt_command_returns_to_tracking_from_slew`.

- `test_halt_from_goto_returns_to_tracking`
  Проверяет инвариант: halt во время goto корректно возвращает tracking state.
  Основа: `test_halt_command_returns_to_tracking_from_goto`.

- `test_ra_guide_rates_follow_command_magnitude_and_direction`
  Проверяет инвариант: команды RA guide на полном стеке меняют скорость в верную сторону и величину.
  Основа: `test_coordinate_system_guide_ra_rates`.

- `test_coordinate_system_move_directions_match_mount_coordinates`
  Проверяет инвариант: east/west/north/south действительно двигают mount в ожидаемых направлениях.
  Основа: `test_coordinate_system_slew_directions`.

### `hw/test_polar_compensator_hw.py`

- `test_polar_compensator_stays_disabled_after_non_settled_guides`
  Проверяет инвариант: разнородные guide pulses не должны включать guiding mode.
  Основа: `test_polar_compensator_stays_disabled_after_different_guides`.

- `test_polar_compensator_enters_guiding_after_settled_guides`
  Проверяет инвариант: после серии устойчивых guide corrections компенсатор входит в guiding mode.
  Основа: `test_polar_compensator_enters_guiding_after_settled_guides`.

- `test_polar_compensator_resets_after_guiding_and_new_non_settled_guides`
  Проверяет инвариант: после выхода в guiding новый рассинхрон должен сбрасывать состояние.
  Основа: `test_polar_compensator_resets_after_guiding_then_different_guides`.

## Новые тесты именно для `sky.motor`-адаптеров

Этих тестов в корне сейчас ещё нет, но они прямо следуют из нового API `Motor`.

### `unit/test_motor_stop_semantics.py`

- `test_axis_retries_command_after_motor_stop_require`
  Проверяет инвариант: если `Motor` просит stop через `MotorStopRequire`, `Axis` сначала вызывает `wait_till_stop()`, потом повторяет операцию.

- `test_axis_restores_tracking_after_stop_required_during_set_position`
  Проверяет инвариант: если stop понадобился во время `set_position()`, ось не теряет tracking state.

### `unit/test_skywatcher_motor_adapter.py`

- `test_status_maps_mount_state_to_motor_status`
  Проверяет инвариант: локальный `SkyWatcherMotor.status()` правильно отображает idle/run/target.

- `test_set_direction_requires_stop_while_running`
  Проверяет инвариант: смена направления на движущемся SkyWatcher adapter бросает `MotorStopRequire`.

- `test_set_microsteps_requires_stop_while_running`
  Проверяет инвариант: смена microsteps на движущемся adapter бросает `MotorStopRequire`.

- `test_run_rejects_mode_mismatch`
  Проверяет инвариант: `run()` не стартует, если перед этим не выставлен нужный motion mode.

### `unit/test_tmc2209_motor_adapter.py`

- `test_status_maps_phase_and_mode_to_motor_status`
  Проверяет инвариант: локальный `TMC2209Motor.status()` правильно отображает phase/mode в `MotorStatus`.

- `test_set_direction_requires_stop_while_running`
  Проверяет инвариант: смена направления на движущемся adapter бросает `MotorStopRequire`.

- `test_set_microsteps_requires_stop_while_running`
  Проверяет инвариант: смена microsteps на движущемся adapter бросает `MotorStopRequire`.

- `test_run_rejects_target_without_target_mode`
  Проверяет инвариант: `run()` не запускает target move, пока режим не переключён в target.
