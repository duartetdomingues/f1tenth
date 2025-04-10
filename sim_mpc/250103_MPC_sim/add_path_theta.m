function ref_path = add_path_theta(ref_path, pitch_tag)

theta = ones(length(ref_path),1);

switch pitch_tag
    case 'flat'
        theta(:) = 0;
    case 'uphill5'
        theta(:) = 5;
    case 'uphill10'
        theta(:) = 10;
    case 'downhill5'
        theta(:) = -5;
    case 'downhill10'
        theta(:) = -10;
    case 'updown'
        theta(1:round(length(theta)/3)) = 5.71;
        theta(round(length(theta)/3):end-round(length(theta)/3)) = 0;
        theta(end-round(length(theta)/3):end) = -5.71;
end

ref_path(:,6) = theta;

end