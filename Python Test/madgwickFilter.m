function filtered_data = madgwickFilter(table, beta, dt)
    % Extract sensor data
    acc  = [table.ax, table.ay, table.az];   % in g's
    gyro = [table.gx, table.gy, table.gz];   % in dps

    % Convert gyro from dps -> rad/s (required for filter)
    gyro = deg2rad(gyro);

    % Initialize Madgwick filter
    madgwick = MadgwickAHRS("SamplePeriod", dt, "Beta", beta);

    % Preallocate outputs
    N = height(table);
    q = zeros(N, 4);
    acc_filtered  = zeros(N, 3);
    gyro_filtered = zeros(N, 3);

    % Run filter
    for i = 1:N
        madgwick.UpdateIMU(gyro(i, :), acc(i, :));
        quat = madgwick.Quaternion;
        q(i, :) = quat;

        % Rotation matrix from quaternion (world -> body frame)
        R = quat2rotm(q);

        % Rotate body-frame data to global-frame
        acc_filtered(i, :)  = (R * acc(i, :)')';
        gyro_filtered(i, :) = (R * gyro(i, :)')';
    end

    % Convert filtered gyro data back to degrees.
    gyro_filtered_dps = rad2deg(gyro_filtered);

    % Generate output table
    A = [table.d, acc_filtered, gyro_filtered_dps];

    filtered_data = array2table(A, "VariableNames", ...
        {'d', 'ax', 'ay', 'az', 'gx', 'gy', 'gz'});
end


function R = quat2rotm(q)
    % Normalize quaternion
    q = q / norm(q);

    % Extract vector components.
    w = q(1);
    x = q(2);
    y = q(3);
    z = q(4);

    % Compute R
    R = [1 - 2*(y^2 + z^2),     2*(x*y - z*w),     2*(x*z + y*w);
             2*(x*y + z*w), 1 - 2*(x^2 + z^2),     2*(y*z - x*w);
             2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x^2 + y^2)];
end