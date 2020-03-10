start:
	# чистим кэш
	rm -f log.csv
	# компилируем
	/usr/bin/g++ -g "./src/main.cpp" -o "./src/main"
	# запускаем что получилось
	./src/main