% A robot on the Clunkyard core's Mercury binding. The core owns tracking,
% geometry and protocol handling; this strategy chooses the tactics.
% Bearing is earned: a scan contact's bearing is off by up to its
% bearing error (degrees, a fixed share of the arc), so a wide beam finds a
% robot and a narrow one aims at it. The robot narrows the beam, holds it
% on the target and aims along a line fitted through its recent sightings.
% Contacts within 120 units are exact.
% Clunkyard robot API 2.0.0: one core under every language, the same
% answers everywhere. Facts (a call shows its arguments):
% match: mode ruleset control robots is_3d tick_rate max_ticks
%   robot_id(slot)
% clock: tick seconds remaining_ticks
% arena: size gravity terrain_kind ground_height(x y)
% me: health position velocity heading pitch roll angular_velocity
%   attitude grounded altitude wall_contact applied hull_mounts
%   to_mount(heading pitch) from_mount(heading pitch) mount_toward(point)
% battery: charge capacity regen sustainable_drive compute
%   drive_cost(fraction)
% drive: hover_lift
% gun: aim_now cooldown_ticks spread max_power damage_scale
%   can_fire(power reserve) shot_speed(power) shot_cost(power)
%   reload_ticks(power) shot_damage(power) power_for_speed(speed)
%   power_for_flight_time(range seconds) lead(contact power)
%   lead_point(position velocity power)
% scanner: last proximity_range reach(arc) arc_for_reach(range) is_on
% sensors: contacts nearest cues contact(slot) position_of(contact)
% tracker: cue nearest_bearing_rate velocity(slot) position(slot)
%   history(slot) last_seen(slot)
% nav: zone zone.margin(point) zone.contains(point margin)
%   zone.distance_from(point) storm storm.margin(point)
%   storm.contains(point margin) storm.distance_from(point) hill
%   hill.margin(point) hill.contains(point margin)
%   hill.distance_from(point) wall_margin() pickups nearest_pickup(kind)
%   field_distance(a b)
% score: me best armour
% angles: normalize(degrees) angle_diff(target current)
%   bearing_to(from to) elevation_to(from to) direction(heading pitch)
%   distance(a b) to_chassis(heading pitch attitude)
%   from_chassis(heading pitch attitude) clamp(value low high)
% raw: number(path) text(path) has(path) flag(path) count(path) line()
% Controls: drive.set(throttle turn pitch_turn)
%   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
%   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
%   scanner.power(on) tracker.configure(smoothing forget_ticks cue_ticks)
%   status.set(text value) debug.log(line) raw.extra(json)
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
:- import_module bool, clunkyard, float, int, list, maybe, math, string.

:- func search_arc = float.
search_arc = 40.0.      % degrees: sweeps the circle in half a second
:- func hold_width = float.
hold_width = 60.0.      % units across at the target: it stays in the beam from tick to tick
:- func min_arc = float.
min_arc = 6.0.          % degrees: the narrowest the beam holds
:- func window = int.
window = 12.            % ticks of sightings the aim averages
:- func sightings = int.
sightings = 8.          % narrow sightings in the window before a shot
:- func look_every = int.
look_every = 40.        % ticks between looks around the target, for a nearer robot
:- func reacquire = int.
reacquire = 10.         % ticks to look again where a lost target was

% Dodges taken, and the tick the current one ends.
:- mutable(dodges, int, 0, ground, [untrailed, attach_to_io_state]).
:- mutable(dodge_end, int, 0, ground, [untrailed, attach_to_io_state]).
% The last target's slot, for looking again where it was lost.
:- mutable(slot, int, 0, ground, [untrailed, attach_to_io_state]).

% One sighting: its tick, its bearing error and where it put the robot.
:- type reading ---> reading(at :: int, err :: float, px :: float, py :: float, pz :: float).
% A fitted line: the sightings it rests on, where the robot is now and how fast it moves.
:- type sightline ---> sightline(count :: int, lx :: float, ly :: float, lz :: float, vx :: float, vy :: float, vz :: float).

:- pred read_readings(cy_sighting::in, int::in, int::in, list(reading)::out, io::di, io::uo) is det.
read_readings(History, I, N, Readings, !IO) :-
    (if I >= N then Readings = []
    else
        sighting_at(History, I, S, !IO), sighting_tick(S, Tick, !IO), sighting_bearing_error(S, Error, !IO),
        sighting_position(S, P, !IO), vec3_x(P, X, !IO), vec3_y(P, Y, !IO), vec3_z(P, Z, !IO),
        read_readings(History, I + 1, N, Rest, !IO), Readings = [reading(Tick, Error, X, Y, Z) | Rest]).

% Axis 0, 1 and 2 are x, y and z; 3 is the tick.
:- func value(int, reading) = float.
value(Axis, R) = (if Axis = 0 then R ^ px else if Axis = 1 then R ^ py else if Axis = 2 then R ^ pz else float.float(R ^ at)).
:- func total(int, list(reading)) = float.
total(_, []) = 0.0.
total(Axis, [R | Rs]) = value(Axis, R) + total(Axis, Rs).
% The sum of (tick - t0) * (value - mean) over the sightings; with Axis 3 and mean t0, their spread.
:- func around(int, float, float, list(reading)) = float.
around(_, _, _, []) = 0.0.
around(Axis, T0, Mean, [R | Rs]) = (value(3, R) - T0) * (value(Axis, R) - Mean) + around(Axis, T0, Mean, Rs).
:- func recent(int, list(reading)) = list(reading).
recent(_, []) = [].
recent(Since, [R | Rs]) = (if R ^ at > Since then [R | recent(Since, Rs)] else recent(Since, Rs)).
:- func narrow(float, list(reading)) = list(reading).
narrow(_, []) = [].
narrow(Limit, [R | Rs]) = (if R ^ err =< Limit then [R | narrow(Limit, Rs)] else narrow(Limit, Rs)).
:- func least(list(reading)) = float.
least([]) = 0.0.
least([R | Rs]) = (if Rs = [] then R ^ err else float.min(R ^ err, least(Rs))).

% A straight line through the narrowest of the last window ticks' sightings:
% where the robot is now and how fast it moves. Each sighting is off by its
% bearing error; the line averages that out.
:- func fit(int, int, list(reading)) = maybe(sightline).
fit(Tick, Rate, Readings) = Fit :-
    Recent = recent(Tick - window, Readings),
    Seen = narrow(1.5 * least(Recent), Recent),                 % the narrow ones
    (if Recent = [_, _ | _], Seen = [_, _ | _] then
        N = float.float(list.length(Seen)), T0 = total(3, Seen) / N, Spread = around(3, T0, T0, Seen),
        MeanX = total(0, Seen) / N, SlopeX = around(0, T0, MeanX, Seen) / Spread,
        MeanY = total(1, Seen) / N, SlopeY = around(1, T0, MeanY, Seen) / Spread,
        MeanZ = total(2, Seen) / N, SlopeZ = around(2, T0, MeanZ, Seen) / Spread,
        Ahead = float.float(Tick) - T0, PerSecond = float.float(Rate),
        Fit = yes(sightline(list.length(Seen), MeanX + SlopeX * Ahead, MeanY + SlopeY * Ahead, MeanZ + SlopeZ * Ahead,
            SlopeX * PerSecond, SlopeY * PerSecond, SlopeZ * PerSecond))
    else Fit = no).

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
:- pred heard_shot(robot::in, bool::out, io::di, io::uo) is det.
heard_shot(R, Shot, !IO) :-
    new_contact_array(64, Cues, !IO), new_count_ref(CountRef, !IO),
    cy_sensors_cues(R, Cues, 64, CountRef, S, !IO), check(R, S, !IO), read_count_ref(CountRef, Count, !IO),
    muzzle_among(Cues, 0, int.min(Count, 64), Shot, !IO).
:- pred muzzle_among(cy_contact::in, int::in, int::in, bool::out, io::di, io::uo) is det.
muzzle_among(Cues, I, N, Shot, !IO) :-
    (if I >= N then Shot = no
    else contact_at(Cues, I, Cue, !IO), contact_source(Cue, Source, !IO),
        (if Source = cy_source_muzzle then Shot = yes else muzzle_among(Cues, I + 1, N, Shot, !IO))).
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
        contact_slot(Target, TargetSlot, !IO), set_slot(TargetSlot, !IO), contact_bearing_error(Target, BearingError, !IO),
        steer(R, Bearing, Heading, 45.0, Turn, !IO), steer(R, Elevation, Pitch, 45.0, Climb, !IO), drive(R, Throttle, Turn, Climb, !IO),
        % Halve the beam each tick until it is hold_width across at the target:
        % narrow enough for a precise bearing, wide enough to keep it in.
        Hold = float.max(min_arc, 2.0 * math.atan2(hold_width / 2.0, Range) * 180.0 / math.pi),
        new_scan(LastScan, !IO), cy_scanner_last(R, LastScan, LSS, !IO), check(R, LSS, !IO), scan_arc(LastScan, LastArc, !IO),
        % In a crowd a narrow beam sees one robot. Every look_every ticks,
        % widen it for a tick: a nearer robot it finds becomes the target.
        Arc = (if Tick mod look_every = 0 then search_arc else float.max(Hold, float.min(search_arc, LastArc / 2.0))),
        scan(R, Bearing, Arc, Elevation, !IO),
        new_float_ref(PowerRef, !IO), cy_gun_power_for_flight_time(R, Range, 1.0, PowerRef, PWS, !IO), check(R, PWS, !IO), read_float_ref(PowerRef, Power, !IO),
        % An exact contact: cy_gun_lead is the intercept if the target holds course.
        % A noisy one: lead the fitted line, and fire only once it rests on enough sightings.
        new_solution(Aim, !IO),
        (if BearingError > 0.0 then
            new_int_ref(RateRef, !IO), cy_match_tick_rate(R, RateRef, RTS, !IO), check(R, RTS, !IO), read_int_ref(RateRef, Rate, !IO),
            new_sighting_array(32, History, !IO), new_count_ref(HistoryCount, !IO),
            cy_tracker_history(R, TargetSlot, History, 32, HistoryCount, HS, !IO), check(R, HS, !IO), read_count_ref(HistoryCount, Recorded, !IO),
            read_readings(History, 0, int.min(Recorded, 32), Readings, !IO),
            (if fit(Tick, Rate, Readings) = yes(Fit) then
                set_vec3(Fit ^ lx, Fit ^ ly, Fit ^ lz, Where, !IO), set_vec3(Fit ^ vx, Fit ^ vy, Fit ^ vz, Velocity, !IO),
                cy_gun_lead_point(R, Where, Velocity, Power, Aim, AS, !IO),
                Ready = (if Fit ^ count >= sightings then yes else no)
            else AS = cy_none, Ready = no)
        else cy_gun_lead(R, Target, Power, Aim, AS, !IO), Ready = yes),
        (if AS = cy_ok then solution_heading(Aim, AimHeading, !IO), solution_pitch(Aim, AimPitch, !IO) else AimHeading = Bearing, AimPitch = Elevation),
        set(cy_gun_aim_heading, R, AimHeading, !IO), set(cy_gun_aim_pitch, R, AimPitch, !IO),
        new_direction(Now, !IO), cy_gun_aim_now(R, Now, NS, !IO), check(R, NS, !IO), direction_heading(Now, NowHeading, !IO), direction_pitch(Now, NowPitch, !IO),
        new_float_ref(DeltaRef, !IO), cy_angles_angle_diff(AimHeading, NowHeading, DeltaRef, DS, !IO), check(R, DS, !IO), read_float_ref(DeltaRef, Delta, !IO),
        new_int_ref(FireRef, !IO), cy_gun_can_fire(R, Power, 10.0, FireRef, FS, !IO), check(R, FS, !IO), read_int_ref(FireRef, CanFire, !IO),
        (if Ready = yes, float.abs(Delta) < 3.0, float.abs(AimPitch - NowPitch) < 3.0, CanFire \= 0 then cy_gun_fire(R, Fired, !IO), check(R, Fired, !IO), set(cy_gun_power, R, Power, !IO) else true),
        status_value(R, (if Ready = yes then "TRACK" else "NARROW"), Range, !IO)
    else
        get_slot(Slot, !IO),
        (if Slot \= 0 then
            new_int_ref(SeenRef, !IO), cy_tracker_last_seen(R, Slot, SeenRef, SeenStatus, !IO), read_int_ref(SeenRef, SeenAt, !IO),
            Lost = (if SeenStatus = cy_ok, Tick - SeenAt =< reacquire then yes else no)
        else Lost = no),
        new_cue(Cue, !IO), cy_tracker_cue(R, Cue, CueStatus, !IO),
        (if Lost = yes then
            % Lost it: look again where it was last seen, widening the beam, before searching.
            new_vec3(Remembered, !IO), cy_tracker_position(R, Slot, Remembered, RememberedStatus, !IO),
            (if RememberedStatus = cy_ok then Where = Remembered else Where = Position),
            angle(cy_angles_bearing_to, R, Position, Where, Bearing, !IO),
            (if Free = 1 then angle(cy_angles_elevation_to, R, Position, Where, Elevation, !IO) else Elevation = 0.0),
            new_scan(LastScan, !IO), cy_scanner_last(R, LastScan, LSS, !IO), check(R, LSS, !IO), scan_arc(LastScan, LastArc, !IO),
            steer(R, Bearing, Heading, 45.0, Turn, !IO), steer(R, Elevation, Pitch, 45.0, Climb, !IO), drive(R, 0.5, Turn, Climb, !IO),
            scan(R, Bearing, float.min(search_arc, LastArc * 2.0), Elevation, !IO), status(R, "REACQUIRE", !IO)
        else if CueStatus = cy_ok then
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
            Elevation = (if Free = 1 then float.float((Tick div 10) mod 3 - 1) * 35.0 else 0.0), scan(R, Sweep, search_arc, Elevation, !IO), status(R, "SEARCH", !IO)
        )
    ),
    % A simple dodge: vary it, or predict the shooter.
    get_dodges(Taken, !IO), get_dodge_end(Ends, !IO), heard_shot(R, Shot, !IO),
    (if Tick >= Ends, Shot = yes then set_dodges(Taken + 1, !IO), set_dodge_end(Tick + 10, !IO) else true),
    get_dodges(Dodges, !IO), get_dodge_end(DodgeEnd, !IO),
    (if Tick < DodgeEnd then
        % Off the line of fire: left, then right next time (turn is positive left, strafe positive right).
        Left = (if Dodges mod 2 = 1 then yes else no),
        new_int_ref(ControlRef, !IO), cy_match_control(R, ControlRef, CS, !IO), check(R, CS, !IO), read_int_ref(ControlRef, Control, !IO),
        (if Control = cy_control_forces then
            set(cy_drive_throttle, R, 0.5, !IO), set(cy_drive_strafe, R, (if Left = yes then -1.0 else 1.0), !IO)
        else
            set(cy_drive_throttle, R, 1.0, !IO), set(cy_drive_turn, R, (if Left = yes then 1.0 else -1.0), !IO)),
        status_value(R, (if Left = yes then "dodge:left" else "dodge:right"), float.float(Dodges), !IO)
    else true),
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
    else true).
