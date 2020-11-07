%% Defining the OWI Robot Links

% Link lengths
l1 = 0.2, l2 = 0.5, l3 = 0.5, l4 = 0.7;
% Links with DH parameters
L1 = Link('d', l1, 'a', 0, 'alpha', pi/2, 'offset', 0)
L2 = Link('d', 0, 'a', l2, 'alpha', 0, 'offset', 0)
L3 = Link('d', 0, 'a', l3, 'alpha', 0)
L4 = Link('d', 0, 'a', l4, 'alpha', -pi/2, 'offset', 0)

%% Defining Robot
bot = SerialLink([L1 L2 L3 L4], 'name', 'OWI')

bot.n

bot.plot([0 0 0 0])
bot.fkine([1 1 1 1])