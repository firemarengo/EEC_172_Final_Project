function filtered_table = enhancedKalmanFilter(table, dt)
    dist = table.d;     % Measured distance (in)
    ax   = table.ax;    % Acceleration in x (g)
    ax   = ax * 9.81 * (100 / 2.54);    % g -> in/s^2

    N = length(dist);

    % State: [position; velocity]
    x = zeros(2, 1);
    P = eye(2);        % Initial covariance.

    % Process and measurement noise tuning parameters
    Q = [0.1, 0; 0, 1];     % Process noise
    R = 2^2;                % Measurement noise variance (in^2).

    % Preallocations.
    X_out = zeros(N, 2);
    Z_out = zeros(N, 1);

    % Filtering step
    for k = 1:N
        % Prediction
        a = ax(k);
        A = [1, dt; 0, 1];
        B = [0.5*dt^2; dt];
        x_pred = A*x + B*a;
        P_pred = A*P*A' + Q;

        % Measurement
        z = dist(k);
        H = [1, 0];
        y = z - H*x_pred;
        S = H*P_pred*H' + R;
        K = P_pred*H' / S;

        % Update step
        x = x_pred + K*y;
        P = (eye(2) - K*H) * P_pred;

        X_out(k, :) = x';
        Z_out(k) = z;
    end

    % Return table
    array = [X_out(:, 1), table.ax, table.ay, table.az, ...
             table.gx, table.gy, table.gz];

    filtered_table = array2table(array, "VariableNames", ...
        {'d', 'ax', 'ay', 'az', 'gx', 'gy', 'gz'});












    %{
    % Initial state: [position; velocity]
    x0 = [dist(1); 0];  % Start at 1st measurement. Velocity is 0 here.
    P0 = eye(2) * 10;   % Initial uncertainty.

    % Define EKF function-type parameters.
    f = @(x, u) [x(1) + x(2)*dt + 0.5*u*dt^2; ...
                 x(2) + u*dt]; % State transition with an acceleration input.

    h = @(x) x(1);

    % Create EKF itself.
    ekf = trackingEKF(@(x, u) f(x, u), h, x0, 'StateCovariance', P0, ...
                      'ControlInputPort', true, 'MeasurementNoise', 4, ...
                      'ProcessNoise', eye(2) * 0.5);

    % Apply the filter.
    N = length(dist);
    filtered_dist = zeros(N, 1);

    for k = 1:N
        u = ax(k);      % Control input: acceleration in in/s^2.
        z = dist(k);    % Measured distance.

        predict(ekf, u);                    % Prediction step
        corrected = correct(ekf, z);        % Correction step
        filtered_dist(k) = corrected(1);    % Storage step
    end
    
    filtered_table = table(filered_dist, table.ax, table.ay, table.az, ...
                           table.gx, table.gy, table.gz, ...
                           'VariableNames', ...
                           {'d', 'ax', 'ay', 'az', 'gx', 'gy', 'gz'});

    %}