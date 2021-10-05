INFLUX_PASSWD:=........
INFLUX_ORG=homer1
MY_HOSTNAME:=watchtower.massrelay.io
MY_SWAP_SIZE:=8096
LSB_RELEASE:=$(shell lsb_release -cs)


.PHONY: help
help: ## help
	@echo 'targets:'
	@sed -n 's/^\([0-9a-z \-]*\):.*\(##.*\)/\1 \2/p' ${MAKEFILE_LIST} | sort | column -t -c 2 -s ':#'


.PHONY: up
up: dirs ## up run docker services
	docker-compose up

.PHONY: purge
purge: ## purge kill all docker services systemwide!!! not just this docker-compose file
	docker system prune --volumes -f

.PHONY: dontrunme
dontrunme: ## dontrunme DELETE ALL DATA
	rm -rf /opt/homer1/data

# ==============================================================================

.PHONY: grafana-shell
grafana-shell: ## grafana-shell run a shell inside grafana container
	docker-compose exec -u 0 grafana /bin/bash

# ==============================================================================

.PHONY: influx-shell
influx-shell: ## influx-shell run a shell inside influxdb container
	docker-compose exec influxdb /bin/bash

.PHONY: influx-setup
influx-setup: ## influx-setup influxdb initial setup
	docker-compose exec influxdb /bin/bash -c \
		'influx setup -b $(INFLUX_ORG) -o $(INFLUX_ORG) -p$(INFLUX_PASSWD) -uroot -r0 -f'

.PHONY: influx-dashboards
influx-dashboards: ## influx-dashboards import some default influxdb dashboards
	docker-compose exec influxdb /bin/bash -c \
		'influx apply -q --force true -f https://raw.githubusercontent.com/influxdata/community-templates/master/influxdb2_operational_monitoring/influxdb2_operational_monitoring.yml'
	docker-compose exec influxdb /bin/bash -c \
		'influx apply -q --force true -f https://raw.githubusercontent.com/influxdata/community-templates/master/influxdb2_operational_monitoring/influxdb2_cardinality_now.yml'
	docker-compose exec influxdb /bin/bash -c \
		'influx apply -q --force true -f https://raw.githubusercontent.com/influxdata/community-templates/master/raspberry-pi/raspberry-pi-system.yml'
	docker-compose exec influxdb /bin/bash -c \
		'influx apply -q --force true -f https://raw.githubusercontent.com/influxdata/community-templates/master/speedtest/speedtest.yml'
	# this one is broken:
	# https://raw.githubusercontent.com/influxdata/community-templates/master/influxdb2_oss_metrics/influxdb2_oss_metrics.yml
	docker-compose exec influxdb /bin/bash -c \
		'influx bucket create --name oss_metrics -r 30d'
	docker-compose exec influxdb /bin/bash -c \
		'influx apply -q --force true -f /tmp/my_dashboards/influxdb2_oss_metrics_dashboard.yaml'
	@echo "please manually create the scapper from URL=http://watchtower.massrelay.io:8086/metrics into bucket oss_metrics"

.PHONY: influx-get-token
influx-get-token: ## influx-get-token get root token from influxdb
	docker-compose exec influxdb /bin/bash -c "influx auth list --hide-headers | grep 'root.*s Token' | awk '{print \$$4}'"

.PHONY: telegraf-install
telegraf-install: ## telegraf-install add telegraf debian repo and install it
	rm -rf /etc/apt/sources.list.d/influxdb.list
	curl -sL https://repos.influxdata.com/influxdb.key | apt-key add -
	echo "deb https://repos.influxdata.com/debian $(LSB_RELEASE) stable" |\
	   	tee /etc/apt/sources.list.d/influxdb.list
	apt -y update
	apt -y upgrade
	apt install -y telegraf

.PHONY: telegraf-setup
telegraf-setup: ## telegraf-setup setup telegraf
	usermod -a -G video telegraf
	echo INFLUX_HOST=http://$(MY_HOSTNAME):8086 > /tmp/telegraf
	echo INFLUX_ORG=$(INFLUX_ORG) >> /tmp/telegraf
	echo -n INFLUX_TOKEN= >> /tmp/telegraf
	docker-compose exec influxdb /bin/bash -c "influx auth list --hide-headers | grep 'root.*s Token' | awk '{print \$$4}'" >> /tmp/telegraf
	cat /tmp/telegraf | tr -d '\r' > /etc/default/telegraf
	echo >> /etc/default/telegraf
	cat /opt/homer1/conf/telegraf.conf > /etc/telegraf/telegraf.conf
	rm -rf /tmp/telegraf
	systemctl enable --now telegraf
	systemctl restart telegraf

.PHONY: telegraf-logs
telegraf-logs: ## telegraf-logs show journalctl logs of telegraf
	journalctl -xefu telegraf

.PHONY: telegraf-restart
telegraf-restart: ## telegraf-restart restart telegraf service
	systemctl restart telegraf

.PHONY: telegraf-kill
telegraf-kill: ## telegraf-kill kill -9 the telegraf service
	pkill -9 telegraf

# ==============================================================================

.PHONY: speedtest-install
speedtest-install: ## install the speedtest.net cli
	curl -s https://install.speedtest.net/app/cli/install.deb.sh | bash
	apt install -y speedtest


# ==============================================================================

.PHONY: node-exporter-install
node-exporter-install: ## node-exporter-install install node exporter from prometheus
	rm -rf /usr/bin/node_exporter
	curl -L 'https://github.com/prometheus/node_exporter/releases/download/v1.2.2/node_exporter-1.2.2.linux-arm64.tar.gz' -o /tmp/node_exporter.tgz
	tar xf /tmp/node_exporter.tgz -C /tmp
	rm -rf /tmp/node_exporter.tgz
	mv /tmp/node_exporter*/node_exporter /usr/bin
	rm -rf /tmp/node_exporter*
	chmod +x /usr/bin/node_exporter

.PHONY: node-exporter-setup
node-exporter-setup: ## node-exporter-setup setup node exporter
	cp conf/node_exporter.service /etc/systemd/system/node_exporter.service
	systemctl daemon-reload
	systemctl enable --now node_exporter
	systemctl restart node_exporter

.PHONY: node-exporter-logs
node-exporter-logs: ## node-exporter-logs node exporter logs
	journalctl -xefu node_exporter

# ==============================================================================

.PHONY: homer1-service-setup
homer1-service-setup: ## homer1-service-setup setup homer1 systemd service file
	cp conf/homer1.service /etc/systemd/system/homer1.service
	systemctl daemon-reload
	systemctl enable --now homer1

.PHONY: homer1-service-start
homer1-service-start: ## homer1-service-start start homer1 systemd service 
	systemctl start homer1

.PHONY: homer1-service-logs
homer1-service-logs: ## homer1-service-logs homer1 logs
	journalctl -xefu homer1

# ==============================================================================
#
.PHONY: hostname
hostname: ## hostname change hostname to whatever $MY_HOSTNAME is
	hostnamectl set-hostname ${MY_HOSTNAME}
	echo "127.0.0.1 ${MY_HOSTNAME}" >> /etc/hosts

.PHONY: swap
swap: ## swap increase swap size to whatever $MY_SWAP_SIZE is
	dphys-swapfile swapoff
	sed -i "s/CONF_SWAPSIZE=.*/CONF_SWAPSIZE=${MY_SWAP_SIZE}/g" /etc/dphys-swapfile
	sed -i "s/#CONF_MAXSWAP=2048/CONF_MAXSWAP=${MY_SWAP_SIZE}/g" /etc/dphys-swapfile
	dphys-swapfile setup
	dphys-swapfile swapon
	ln -s /dev/shm /a

.PHONY: tools
tools: ## tools install some debian apt goodies
	rm -rf /usr/share/keyrings/docker-archive-keyring.gpg
	rm -rf /etc/apt/sources.list.d/docker.list
	curl -fsSL https://download.docker.com/linux/debian/gpg \
		| gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
	echo "deb [arch=arm64 signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] \
		https://download.docker.com/linux/debian $(LSB_RELEASE) stable" | \
		tee /etc/apt/sources.list.d/docker.list > /dev/null
	apt update -y
	apt upgrade -y
	apt autoremove -y
	apt install -y \
		apt-transport-https \
		ca-certificates \
		curl \
		gnupg \
		lsb-release \
		vim \
		axel \
		git \
		python3-pip \
		tree \
		mlocate
	apt install -y \
		docker-ce \
		docker-ce-cli \
		containerd.io \
		docker-compose
	cp /opt/homer1/undervoltage.sh /usr/bin/

.PHONY: dirs
dirs: ## dirs creare dirs required by docker-compose volumes
	mkdir -p /opt/homer1/data/grafana_share/
	mkdir -p /opt/homer1/data/grafana_lib/
	mkdir -p /opt/homer1/data/grafana_etc/
	mkdir -p /opt/homer1/data/influxdbv2_root/
	mkdir -p /opt/homer1/data/influxdbv2_lib/
	mkdir -p /opt/homer1/data/prometheus_etc/
	mkdir -p /opt/homer1/data/prometheus_data/
	mkdir -p /opt/homer1/data/influxdbv2_dashboards/
	chmod -R 777 /opt/homer1/data/grafana_share/
	chmod -R 777 /opt/homer1/data/grafana_lib/
	chmod -R 777 /opt/homer1/data/grafana_etc/
	chmod -R 777 /opt/homer1/data/influxdbv2_root/
	chmod -R 777 /opt/homer1/data/influxdbv2_lib/
	chmod -R 777 /opt/homer1/data/prometheus_etc/
	chmod -R 777 /opt/homer1/data/prometheus_data/
	chmod -R 777 /opt/homer1/data/influxdbv2_dashboards/
	cp -n /opt/homer1/conf/prometheus.yml /opt/homer1/data/prometheus_etc/
	cp -n /opt/homer1/conf/influxdb2_oss_metrics_dashboard.yaml /opt/homer1/data/influxdbv2_dashboards/
	cp -n -r /opt/homer1/conf/grafana/* /opt/homer1/data/grafana_etc/

.PHONY: motd
motd:
	echo > /etc/motd

