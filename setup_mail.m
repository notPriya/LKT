if ~strcmp(getpref('Internet', 'SMTP_Username'), 'pdeo')
    % Setup the SMTP stuff.
    password = input('Enter Password:  ', 's');
    setpref('Internet', 'E_mail', 'pdeo@andrew.cmu.edu')
    setpref('Internet', 'SMTP_Server', 'smtp.andrew.cmu.edu');
    setpref('Internet', 'SMTP_Username', 'pdeo');
    setpref('Internet', 'SMTP_Password', password);
    % Dont save the password!
    clear password;
    % Clear screen to get rid of the password.
    clc;
    % Pause matlab so user can clear password.
    input('Please clear password from command history.');
end
% Setup the SSL connection.
props = java.lang.System.getProperties;
if isempty(props.getProperty('mail.smtp.auth'))
    props.setProperty('mail.smtp.auth','true');
    props.setProperty('mail.smtp.socketFactory.class', ...
                      'javax.net.ssl.SSLSocketFactory');
    props.setProperty('mail.smtp.socketFactory.port','465'); 
end
clear props;
