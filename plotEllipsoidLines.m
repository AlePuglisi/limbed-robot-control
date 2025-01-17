function h = plotEllipsoidLines(E, center, color)
    % plotEllipsoidLines - Plots an ellipsoid as a set of 3D lines defined by x'Ex = 1.
    %
    % Syntax:
    %   h = plotEllipsoidLines(E)
    %   h = plotEllipsoidLines(E, center)
    %   h = plotEllipsoidLines(E, center, color)
    %
    % Inputs:
    %   E      - 3x3 positive definite matrix defining the ellipsoid.
    %   center - (Optional) 3x1 vector specifying the ellipsoid's center.
    %            Default is [0; 0; 0].
    %   color  - (Optional) String or RGB triplet specifying the line color.
    %            Default is 'blue'.
    %
    % Outputs:
    %   h      - Array of line handles for the plotted ellipsoid lines.
    %
    % Example:
    %   E = [4 0 0; 0 1 0; 0 0 0.25];
    %   center = [1; 2; 3];
    %   color = 'red';
    %   h = plotEllipsoidLines(E, center, color);

    % Validate inputs
    if nargin < 2 || isempty(center)
        center = [0; 0; 0];
    end
    if nargin < 3 || isempty(color)
        color = 'blue';
    end
    if size(E, 1) ~= 3 || size(E, 2) ~= 3
        error('Matrix E must be 3x3.');
    end
    if numel(center) ~= 3
        error('Center must be a 3x1 vector.');
    end

    % Eigenvalue decomposition of E
    [V, D] = eig(E);
    if any(diag(D) <= 0)
        error('Matrix E must be positive definite.');
    end

    % Scale the ellipsoid to a unit sphere
    radii = sqrt(1 ./ diag(D));
    [X, Y, Z] = sphere(30); % Generate a unit sphere with 30x30 points

    % Transform the sphere into the ellipsoid
    ellipsoidPoints = V * diag(radii) * [X(:)'; Y(:)'; Z(:)'];

    % Apply translation
    ellipsoidPoints(1, :) = ellipsoidPoints(1, :) + center(1);
    ellipsoidPoints(2, :) = ellipsoidPoints(2, :) + center(2);
    ellipsoidPoints(3, :) = ellipsoidPoints(3, :) + center(3);

    % Reshape points for plotting
    X = reshape(ellipsoidPoints(1, :), size(X));
    Y = reshape(ellipsoidPoints(2, :), size(Y));
    Z = reshape(ellipsoidPoints(3, :), size(Z));

    % Plot the lines in 3D
    hold on;
    h = gobjects(2 * size(X, 1), 1); % Preallocate handles

    % Plot meridians (lines of constant X or Y)
    for i = 1:size(X, 1)
        h(i) = plot3(X(i, :), Y(i, :), Z(i, :), 'Color', color);
    end

    % Plot parallels (lines of constant Z)
    for j = 1:size(X, 2)
        h(size(X, 1) + j) = plot3(X(:, j), Y(:, j), Z(:, j), 'Color', color);
    end
end