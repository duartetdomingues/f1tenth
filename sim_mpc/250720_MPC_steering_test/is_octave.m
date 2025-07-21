function answer = is_octave()
  persistent x;
  if isempty(x)
    x = (exist('OCTAVE_VERSION', 'builtin') ~= 0);
  end
  answer = x;
end
