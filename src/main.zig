const std = @import("std");
const os = std.os;
const math = std.math;
const Vector = std.meta;
const print = std.debug.print;
const stdout_file = std.io.getStdOut().writer();
var bw = std.io.bufferedWriter(stdout_file);
const stdout = bw.writer();

const ROTATION_FACTOR: f32 = 2; // Multiplier of rotation (Speed of rotation)
const SCREEN_RES: u16 = 50; // Resolution of the screen 1 = 2 characters from the ASCII table

const ASCII = ".,-~:;=!*%#$"; // Set of ASCII characters to be used
const AX_STEP: f32 = 0.016; // Angle in radians per one step in X axis
const AY_STEP: f32 = 0.006; // Angle in radians per one step in Y axis
const AZ_STEP: f32 = 0.015; // Angle in radians per one step in Z axis
const TARGET_FRAME_TIME: u32 = 50; // Time between frames in milliseconds 1000/50 = 20 FPS

const RADIUS: f32 = SCREEN_RES / 2;
const SIZE: f32 = @sqrt((RADIUS * RADIUS) * 4 / 3);
const SCREEN_SIZE: f32 = RADIUS * 2;
const SCREEN_POS: f32 = -2 * RADIUS;
const MAX_DISTANCE: f32 = RADIUS * 1.1 - SCREEN_POS;
const MIN_DISTANCE: f32 = -RADIUS - SCREEN_POS;
const RES: u16 = @round(SIZE) + 1;
const POINTS_COUNT = RES * 12; // 12 is the numver of edges in a cube
const TAU: f32 = math.tau;

const Cube = struct {
    points: [POINTS_COUNT]@Vector(3, f32) = undefined,
    POINTS: [POINTS_COUNT]@Vector(3, f32) = undefined,

    pub fn init(self: *Cube) void {
        const theta = TAU / 4;
        const d = SIZE / (RES - 1);
        var edge: [RES]@Vector(3, f32) = undefined;
        var i: u32 = 0;

        // Initial edge
        for ([_]u8{0} ** RES) |_| {
            edge[i] = @Vector(3, f32){ d * @as(f32, @floatFromInt(i)) - SIZE / 2, -SIZE / 2, -SIZE / 2 };
            self.points[i] = edge[i];
            i += 1;
        }

        // Multiply into 4 edges by rotating over x axis
        for ([_]u8{0} ** 3) |_| {
            for ([_]u8{0} ** RES, 0..) |_, edge_i| {
                rotate_x(&edge[edge_i], theta);
                self.points[i] = edge[edge_i];
                i += 1;
            }
        }

        // Rotate over z axis
        for ([_]u8{0} ** RES, 0..) |_, edge_i| {
            rotate_z(&edge[edge_i], theta);
            self.points[i] = edge[edge_i];
            i += 1;
        }

        // Multiply into 4 edges by rotating over y axis
        for ([_]u8{0} ** 3) |_| {
            for ([_]u8{0} ** RES, 0..) |_, edge_i| {
                rotate_y(&edge[edge_i], theta);
                self.points[i] = edge[edge_i];
                i += 1;
            }
        }

        // Rotate over x axis
        for ([_]u8{0} ** RES, 0..) |_, edge_i| {
            rotate_x(&edge[edge_i], theta);
            self.points[i] = edge[edge_i];
            i += 1;
        }

        // Multiply into 4 edges by rotating over y axis
        for ([_]u8{0} ** 3) |_| {
            for ([_]u8{0} ** RES, 0..) |_, edge_i| {
                rotate_z(&edge[edge_i], theta);
                self.points[i] = edge[edge_i];
                i += 1;
            }
        }

        self.POINTS = self.points;
    }

    pub fn rotate(self: *Cube, aX: f32, aY: f32, aZ: f32) void {
        var cosa: f32 = @cos(aZ);
        var sina: f32 = @sin(aZ);

        var cosb: f32 = @cos(aY);
        var sinb: f32 = @sin(aY);

        var cosc: f32 = @cos(aX);
        var sinc: f32 = @sin(aX);

        var aX_x: f32 = cosa * cosb;
        var aX_y: f32 = cosa * sinb * sinc - sina * cosc;
        var aX_z: f32 = cosa * sinb * cosc + sina * sinc;

        var aY_x: f32 = sina * cosb;
        var aY_y: f32 = sina * sinb * sinc + cosa * cosc;
        var aY_z: f32 = sina * sinb * cosc - cosa * sinc;

        var aZ_x: f32 = -sinb;
        var aZ_y: f32 = cosb * sinc;
        var aZ_z: f32 = cosb * cosc;
        var index: u32 = 0;
        for (self.POINTS) |point| {
            var x: f32 = point[0];
            var y: f32 = point[1];
            var z: f32 = point[2];

            var new_x: f32 = aX_x * x + aX_y * y + aX_z * z;
            var new_y: f32 = aY_x * x + aY_y * y + aY_z * z;
            var new_z: f32 = aZ_x * x + aZ_y * y + aZ_z * z;

            self.points[index] = @Vector(3, f32){ new_x, new_y, new_z };
            index += 1;
        }
    }
};

const Camera = struct {
    screen: [SCREEN_RES][SCREEN_RES]f32 = std.mem.zeroes([SCREEN_RES][SCREEN_RES]f32),

    pub fn capture(self: *Camera, cube: *Cube) void {
        var screen_distance_buffer: [SCREEN_RES][SCREEN_RES]f32 = std.mem.zeroes([SCREEN_RES][SCREEN_RES]f32);
        self.screen = std.mem.zeroes([SCREEN_RES][SCREEN_RES]f32);

        for (cube.*.points) |point| {
            var dX: f32 = point[0];
            var dY: f32 = point[1];
            var dZ: f32 = abs(point[2] - SCREEN_POS);
            var distance: f32 = @sqrt(dX * dX + dY * dY + dZ * dZ);

            var sfX: f32 = dX + @as(f32, SCREEN_SIZE) / 2;
            var sfY: f32 = dY + @as(f32, SCREEN_SIZE) / 2;

            var sX: u16 = @intFromFloat(sfX * @as(f32, SCREEN_RES) / @as(f32, SCREEN_SIZE));
            var sY: u16 = @intFromFloat(sfY * @as(f32, SCREEN_RES) / @as(f32, SCREEN_SIZE));
            if (screen_distance_buffer[sX][sY] == 0 or screen_distance_buffer[sX][sY] > distance) {
                screen_distance_buffer[sX][sY] = distance;
                self.screen[sY][sX] = (MIN_DISTANCE - distance) / (MAX_DISTANCE - MIN_DISTANCE) + 1;
            }
        }
    }

    pub fn color_map(p: f32) u8 {
        const r = p;
        const g = 0;
        const b = 1 - p;
        const index: u8 = 16 + (36 * @as(u8, @intFromFloat(r * 5))) + (6 * @as(u8, @intFromFloat(g * 5))) + @as(u8, @intFromFloat(b * 5));
        return index;
    }

    pub fn display(self: *Camera, ascii: *const [12:0]u8) !void {
        try stdout.print("\x1b[2J", .{});
        for (self.screen) |row| {
            for (row) |pixel_value| {
                var char_index: u8 = @intFromFloat(@round(pixel_value * @as(f32, 12)));
                if (char_index == 0) {
                    try stdout.print("  ", .{});
                } else {
                    var char = ascii.*[char_index - 1];
                    try stdout.print("\x1b[38;5;{}m{c}{c}\x1b[0m", .{ color_map(pixel_value), char, char });
                }
            }
            try stdout.print("\n", .{});
        }
        try bw.flush();
    }
};

pub fn rotate_x(vec: *@Vector(3, f32), angle: f32) void {
    const sina = @sin(angle);
    const cosa = @cos(angle);
    const x = vec.*[0];
    const y = vec.*[1];
    const z = vec.*[2];
    vec.* = @Vector(3, f32){ x, y * cosa - z * sina, y * sina + z * cosa };
}

pub fn rotate_y(vec: *@Vector(3, f32), angle: f32) void {
    const sina = @sin(angle);
    const cosa = @cos(angle);
    const x = vec.*[0];
    const y = vec.*[1];
    const z = vec.*[2];
    vec.* = @Vector(3, f32){ x * cosa + z * sina, y, -x * sina + z * cosa };
}

pub fn rotate_z(vec: *@Vector(3, f32), angle: f32) void {
    const sina = @sin(angle);
    const cosa = @cos(angle);
    const x = vec.*[0];
    const y = vec.*[1];
    const z = vec.*[2];
    vec.* = @Vector(3, f32){ x * cosa - y * sina, x * sina + y * cosa, z };
}

pub fn abs(value: f32) f32 {
    if (value < 0) {
        return value * -1;
    } else {
        return value;
    }
}

pub fn timestamp() u32 {
    var ts: os.timespec = undefined;
    os.clock_gettime(os.CLOCK.REALTIME, &ts) catch |err| switch (err) {
        error.UnsupportedClock, error.Unexpected => return 0,
    };
    const nanos = ts.tv_nsec;
    const milis = @divTrunc(nanos, 1000_000);
    var t: u32 = @intCast(milis);
    return t;
}

pub fn main() !void {
    var ax: f32 = 0;
    var ay: f32 = 0;
    var az: f32 = 0;

    var cube = Cube{};
    var camera = Camera{};

    cube.init();

    const TFT_M1 = TARGET_FRAME_TIME - 1;

    while (true) {
        cube.rotate(ax, ay, az);
        camera.capture(&cube);
        try camera.display(ASCII);

        ax += AX_STEP * ROTATION_FACTOR;
        ay += AY_STEP * ROTATION_FACTOR;
        az += AZ_STEP * ROTATION_FACTOR;

        if (ax > TAU) {
            ax -= TAU;
        }
        if (ay > TAU) {
            ay -= TAU;
        }
        if (az > TAU) {
            az -= TAU;
        }

        while (timestamp() == 0) {}
        while (timestamp() % TFT_M1 != 0) {}
        while (timestamp() % TARGET_FRAME_TIME != 0) {}
    }
}
