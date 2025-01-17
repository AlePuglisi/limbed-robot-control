function h_ellipse = plotEllipsoid(E, center, color)
    % plotEllipsoid - Plots an ellipsoid defined by x'Ex = 1 in 3D space.
    %
    % Syntax:
    %   plotEllipsoid(E)
    %   plotEllipsoid(E, center)
    %   plotEllipsoid(E, center, color)
    %
    % Inputs:
    %   E      - 3x3 positive definite matrix defining the ellipsoid.
    %   center - (Optional) 3x1 vector specifying the ellipsoid's center.
    %            Default is [0; 0; 0].
    %   color  - (Optional) String or RGB triplet specifying the surface
    %            color of the ellipsoid. Default is 'blue'.
    %
    % Example:
    %   E = [4 0 0; 0 1 0; 0 0 0.25];
    %   center = [1; 2; 3];
    %   color = [0.5, 0.8, 0.2]; % RGB triplet
    %   plotEllipsoid(E, center, color);

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
    [X, Y, Z] = sphere(50); % Generate a unit sphere with 50x50 points

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

    % Plot the ellipsoid
    h_ellipse = mesh(X, Y, Z, 'FaceAlpha', 0.3, 'FaceColor', color);
end