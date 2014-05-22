function emailResults(password, settings, results)
    setpref('Internet', 'E_mail', 'pdeo@andrew.cmu.edu')
    setpref('Internet', 'SMTP_Server', 'smtp.andrew.cmu.edu');
    setpref('Internet', 'SMTP_Username', 'pdeo');
    setpref('Internet', 'SMTP_Password', password);

    props = java.lang.System.getProperties;
    props.setProperty('mail.smtp.auth','true');
    props.setProperty('mail.smtp.socketFactory.class', ...
                      'javax.net.ssl.SSLSocketFactory');
    props.setProperty('mail.smtp.socketFactory.port','465');

    sendmail('mkimzie+matlab@gmail.com', 'LKT has finished Running', settings, results);
end