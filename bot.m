% A robot on the Clunkyard core's Mercury binding. The core owns tracking,
% geometry and protocol handling; this strategy chooses the tactics.
% Clunkyard robot API 2.0.0: one core under every language, the same
% answers everywhere. Facts (a call shows its arguments):
% match: mode ruleset control robots is_3d tick_rate max_ticks
%   robot_id(slot)
% clock: tick seconds remaining_ticks
% arena: size gravity terrain_kind ground_height(x y)
% me: health position velocity heading pitch roll angular_velocity
%   attitude grounded altitude applied
% battery: charge capacity regen sustainable_drive compute
%   drive_cost(fraction)
% drive: hover_lift
% gun: aim_now cooldown_ticks spread max_power damage_scale
%   can_fire(power reserve) shot_speed(power) shot_cost(power)
%   reload_ticks(power) shot_damage(power) power_for_speed(speed)
%   power_for_flight_time(range seconds) lead(contact power)
%   lead_point(position velocity power)
% scanner: last proximity_range reach(arc) arc_for_reach(range)
% sensors: contacts nearest cues contact(slot) position_of(contact)
% tracker: cue nearest_bearing_rate velocity(slot) position(slot)
%   last_seen(slot)
% nav: zone zone.margin(point) zone.contains(point margin)
%   zone.distance_from(point) storm storm.margin(point)
%   storm.contains(point margin) storm.distance_from(point) hill
%   hill.margin(point) hill.contains(point margin)
%   hill.distance_from(point) pickups nearest_pickup(kind)
%   field_distance(a b)
% score: me best armour
% angles: normalize(degrees) angle_diff(target current)
%   bearing_to(from to) elevation_to(from to) direction(heading pitch)
%   distance(a b) clamp(value low high)
% raw: number(path) text(path) has(path) flag(path) count(path) line()
% Controls: drive.set(throttle turn pitch_turn)
%   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
%   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
%   tracker.configure(smoothing forget_ticks cue_ticks)
%   status.set(text value) raw.extra(json)
% Every control argument is optional; a second call keeps what the first
% set. Absent facts are your language's none. Names are snake_case here;
% your language cases them its own way. Angles are degrees from the +x
% axis; pitch is positive up.
% Manual: https://clunkyard.com/manual#sdk
:- module bot.
:- interface.
:- import_module io.
:- pred main(io::di, io::uo) is det.
:- implementation.
:- import_module clunkyard, float, int, math, uint64.

:- pred number(pred(robot, float_ref, int, io, io)::in(pred(in, in, out, di, uo) is det), robot::in, float::out, io::di, io::uo) is det.
number(Query, R, Value, !IO) :- new_float_ref(Out, !IO), Query(R, Out, Status, !IO), check(R, Status, !IO), read_float_ref(Out, Value, !IO).
:- pred set(pred(robot, float, int, io, io)::in(pred(in, in, out, di, uo) is det), robot::in, float::in, io::di, io::uo) is det.
set(Control, R, Value, !IO) :- Control(R, Value, Status, !IO), check(R, Status, !IO).
:- pred clamp(robot::in, float::in, float::out, io::di, io::uo) is det.
clamp(R, Value, Out, !IO) :- new_float_ref(Ref, !IO), cy_angles_clamp(Value, -1.0, 1.0, Ref, S, !IO), check(R, S, !IO), read_float_ref(Ref, Out, !IO).
:- pred steer(robot::in, float::in, float::in, float::in, float::out, io::di, io::uo) is det.
steer(R, Target, Current, Scale, Out, !IO) :- new_float_ref(Ref, !IO), cy_angles_angle_diff(Target, Current, Ref, S, !IO), check(R, S, !IO), read_float_ref(Ref, Delta, !IO), clamp(R, Delta / Scale, Out, !IO).
:- pred drive(robot::in, float::in, float::in, float::in, io::di, io::uo) is det.
drive(R, Throttle, Turn, Pitch, !IO) :- set(cy_drive_throttle, R, Throttle, !IO), set(cy_drive_turn, R, Turn, !IO), set(cy_drive_pitch_turn, R, Pitch, !IO).
:- pred scan(robot::in, float::in, float::in, float::in, io::di, io::uo) is det.
scan(R, Heading, Arc, Pitch, !IO) :- set(cy_scanner_heading, R, Heading, !IO), set(cy_scanner_arc, R, Arc, !IO), set(cy_scanner_pitch, R, Pitch, !IO).
:- pred status(robot::in, string::in, io::di, io::uo) is det.
status(R, Text, !IO) :- text_buffer(Text, Bytes, N, !IO), cy_status_text(R, Bytes, N, S, !IO), check(R, S, !IO).
:- pred status_value(robot::in, string::in, float::in, io::di, io::uo) is det.
status_value(R, Text, Value, !IO) :- status(R, Text, !IO), set(cy_status_value, R, Value, !IO).
:- pred angle(pred(cy_vec3, cy_vec3, float_ref, int, io, io)::in(pred(in, in, in, out, di, uo) is det), robot::in, cy_vec3::in, cy_vec3::in, float::out, io::di, io::uo) is det.
angle(Query, R, A, B, Value, !IO) :- new_float_ref(Out, !IO), Query(A, B, Out, S, !IO), check(R, S, !IO), read_float_ref(Out, Value, !IO).
:- pred start(robot::in, io::di, io::uo) is det.
start(_, !IO).
main(!IO) :- run(turn, start, !IO).

:- pred turn(robot::in, io::di, io::uo) is det.
turn(R, !IO) :-
    new_vec3(Position, !IO), cy_me_position(R, Position, PS, !IO), check(R, PS, !IO),
    number(cy_me_heading, R, Heading, !IO), number(cy_me_pitch, R, Pitch, !IO),
    new_int_ref(TickRef, !IO), cy_clock_tick(R, TickRef, TS, !IO), check(R, TS, !IO), read_int_ref(TickRef, Tick, !IO),
    new_int_ref(Mode, !IO), cy_match_is_3d(R, Mode, MS, !IO), check(R, MS, !IO), read_int_ref(Mode, Is3d, !IO),
    new_vec3(Gravity, !IO), cy_arena_gravity(R, Gravity, GS, !IO), check(R, GS, !IO), vec3_z(Gravity, GravityZ, !IO),
    Free = (if Is3d \= 0, GravityZ = 0.0 then 1 else 0),
    set(cy_drive_brake, R, 0.2, !IO),
    new_contact(Target, !IO), cy_sensors_nearest(R, Target, TargetStatus, !IO),
    (if TargetStatus = cy_ok then
        contact_range(Target, Range, !IO), contact_bearing(Target, Bearing, !IO), contact_elevation(Target, Elevation, !IO),
        Throttle = (if Range > 350.0 then 0.7 else if Range < 200.0 then -0.4 else 0.2),
        steer(R, Bearing, Heading, 45.0, Turn, !IO), steer(R, Elevation, Pitch, 45.0, Climb, !IO), drive(R, Throttle, Turn, Climb, !IO), scan(R, Bearing, 20.0, Elevation, !IO),
        new_float_ref(PowerRef, !IO), cy_gun_power_for_flight_time(R, Range, 1.0, PowerRef, PWS, !IO), check(R, PWS, !IO), read_float_ref(PowerRef, Power, !IO),
        new_solution(Aim, !IO), cy_gun_lead(R, Target, Power, Aim, AS, !IO),
        (if AS = cy_ok then solution_heading(Aim, AimHeading, !IO), solution_pitch(Aim, AimPitch, !IO) else AimHeading = Bearing, AimPitch = Elevation),
        set(cy_gun_aim_heading, R, AimHeading, !IO), set(cy_gun_aim_pitch, R, AimPitch, !IO),
        new_direction(Now, !IO), cy_gun_aim_now(R, Now, NS, !IO), check(R, NS, !IO), direction_heading(Now, NowHeading, !IO), direction_pitch(Now, NowPitch, !IO),
        new_float_ref(DeltaRef, !IO), cy_angles_angle_diff(AimHeading, NowHeading, DeltaRef, DS, !IO), check(R, DS, !IO), read_float_ref(DeltaRef, Delta, !IO),
        new_int_ref(FireRef, !IO), cy_gun_can_fire(R, Power, 10.0, FireRef, FS, !IO), check(R, FS, !IO), read_int_ref(FireRef, CanFire, !IO),
        (if float.abs(Delta) < 3.0, float.abs(AimPitch - NowPitch) < 3.0, CanFire \= 0 then cy_gun_fire(R, Fired, !IO), check(R, Fired, !IO), set(cy_gun_power, R, Power, !IO) else true),
        status_value(R, "TRACK", Range, !IO)
    else
        new_cue(Cue, !IO), cy_tracker_cue(R, Cue, CueStatus, !IO),
        (if CueStatus = cy_ok then
            cue_position(Cue, Point, !IO), angle(cy_angles_bearing_to, R, Position, Point, Bearing, !IO), angle(cy_angles_distance, R, Position, Point, Distance, !IO),
            (if Free = 1 then angle(cy_angles_elevation_to, R, Position, Point, Elevation, !IO) else Elevation = 0.0),
            cue_bearing_error(Cue, Error, !IO), Spread = float.min(Error, 30.0), Across = float.float(Tick mod 5 - 2) / 2.0 * Spread,
            Up = (if Free = 1 then float.float((Tick div 5) mod 3 - 1) * Spread / 2.0 else 0.0),
            steer(R, Bearing, Heading, 45.0, Turn, !IO), steer(R, Elevation, Pitch, 45.0, Climb, !IO), drive(R, 0.7, Turn, Climb, !IO),
            new_float_ref(ArcRef, !IO), cy_scanner_arc_for_reach(R, Distance + 100.0, ArcRef, ARS, !IO), check(R, ARS, !IO), read_float_ref(ArcRef, Arc, !IO), scan(R, Bearing + Across, Arc, Elevation + Up, !IO), status_value(R, "CUE", Distance, !IO)
        else
            new_scan(Last, !IO), cy_scanner_last(R, Last, LS, !IO), check(R, LS, !IO), scan_heading(Last, LastHeading, !IO),
            new_float_ref(SweepRef, !IO), cy_angles_normalize(LastHeading + 36.0, SweepRef, SS, !IO), check(R, SS, !IO), read_float_ref(SweepRef, Sweep, !IO),
            steer(R, 0.0, Pitch, 45.0, Climb, !IO), drive(R, 0.5, 0.33, Climb, !IO),
            Elevation = (if Free = 1 then float.float((Tick div 10) mod 3 - 1) * 35.0 else 0.0), scan(R, Sweep, 40.0, Elevation, !IO), status(R, "SEARCH", !IO)
        )
    ),
    number(cy_battery_charge, R, Charge, !IO), number(cy_battery_capacity, R, Capacity, !IO),
    (if Charge < Capacity / 4.0 then number(cy_drive_get_throttle, R, ThrottleNow, !IO), set(cy_drive_throttle, R, ThrottleNow * 0.5, !IO) else true),
    new_storm(Storm, !IO), cy_nav_storm(R, Storm, StormStatus, !IO), new_float_ref(MarginRef, !IO),
    (if StormStatus = cy_ok then
        cy_nav_storm_margin(R, Position, MarginRef, SMS, !IO), read_float_ref(MarginRef, Margin, !IO),
        (if SMS = cy_ok, Margin > -60.0 then storm_center(Storm, Center, !IO), angle(cy_angles_bearing_to, R, Center, Position, Away, !IO), steer(R, Away, Heading, 30.0, StormTurn, !IO), set(cy_drive_throttle, R, 0.8, !IO), set(cy_drive_turn, R, StormTurn, !IO) else true)
    else true),
    new_zone(Zone, !IO), cy_nav_zone(R, Zone, ZoneStatus, !IO),
    (if ZoneStatus = cy_ok then
        cy_nav_zone_margin(R, Position, MarginRef, ZMS, !IO), read_float_ref(MarginRef, ZoneMargin, !IO),
        (if ZMS = cy_ok, ZoneMargin < 60.0 then
            zone_center(Zone, CenterZ, !IO), zone_shape(Zone, Shape, !IO), angle(cy_angles_bearing_to, R, Position, CenterZ, Back, !IO),
            (if Shape = cy_shape_sphere then angle(cy_angles_elevation_to, R, Position, CenterZ, ZoneElevation, !IO) else ZoneElevation = 0.0),
            steer(R, Back, Heading, 30.0, ZoneTurn, !IO), clamp(R, (ZoneElevation - Pitch) / 30.0, ZoneClimb, !IO), drive(R, 0.8, ZoneTurn, ZoneClimb, !IO)
        else true)
    else true),
    % Only the arena's rate calibration sends this; safe to delete.
    text_buffer("state.calibration.work", Path, PathLength, !IO), new_float_ref(WorkRef, !IO), cy_raw_number(R, Path, PathLength, WorkRef, WS, !IO), read_float_ref(WorkRef, Work, !IO),
    (if WS = cy_ok, Work >= 1.0, Work = float.float(float.floor_to_int(Work)) then calibration_work(float.floor_to_int(Work), Tick, Out), status_value(R, "WORK", float.float(Out), !IO) else true).

% The calibration kernel, xorshift-sqrt v1: work rounds of a 64-bit
% xorshift and a square-root sum.
:- pred calibration_work(int::in, int::in, int::out) is det.
calibration_work(Work, Tick, Out) :-
    calibration_rounds(Work, 88172645463325252u64 `xor` uint64.cast_from_int(Tick), X, 0.0, Sum),
    Out = (float.floor_to_int(Sum) + uint64.cast_to_int(X >> 40)) mod 1000000.
:- pred calibration_rounds(int::in, uint64::in, uint64::out, float::in, float::out) is det.
calibration_rounds(N, !X, !Sum) :-
    (if N > 0 then
        !:X = !.X `xor` (!.X << 13), !:X = !.X `xor` (!.X >> 7), !:X = !.X `xor` (!.X << 17),
        !:Sum = !.Sum + math.sqrt(float.cast_from_uint64(!.X >> 11)), calibration_rounds(N - 1, !X, !Sum)
    else true).
