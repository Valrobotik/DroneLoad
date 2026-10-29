#!/bin/bash
# GUI Gazebo servie dans le navigateur par KasmVNC (http://localhost:6901), sans mot de passe :
# le port n'est publié que sur la machine locale.
set -e
export DISPLAY=:1
Xvnc :1 -geometry 1600x900 -depth 24 -interface 0.0.0.0 -websocketPort 6901 \
    -httpd /usr/share/kasmvnc/www -sslOnly 0 -disableBasicAuth -SecurityTypes None &
sleep 2
# Fenêtres maximisées (la page web s'adapte à la taille du navigateur)
mkdir -p ~/.config/openbox
cat > ~/.config/openbox/rc.xml <<'XML'
<openbox_config xmlns="http://openbox.org/3.4/rc">
  <applications><application class="*"><maximized>yes</maximized><decor>no</decor></application></applications>
</openbox_config>
XML
openbox &
exec gz sim -g
