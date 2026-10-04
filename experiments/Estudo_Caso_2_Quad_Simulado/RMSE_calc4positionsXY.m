
RMSE(1,1) = sqrt(mean((X.Data(:,1) - X.Data(:,2)).^2));
RMSE(2,1) = sqrt(mean((X.Data(:,1) - X.Data(:,3)).^2));
RMSE(3,1) = sqrt(mean((X.Data(:,1) - X.Data(:,4)).^2));
RMSE(4,1) = sqrt(mean((X.Data(:,3) - X.Data(:,4)).^2));

RMSE(1,2) = sqrt(mean((Y.Data(:,1) - Y.Data(:,2)).^2));
RMSE(2,2) = sqrt(mean((Y.Data(:,1) - Y.Data(:,3)).^2));
RMSE(3,2) = sqrt(mean((Y.Data(:,1) - Y.Data(:,4)).^2));
RMSE(4,2) = sqrt(mean((Y.Data(:,3) - Y.Data(:,4)).^2));

RMSE