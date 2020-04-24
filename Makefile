build:
	# чистим кэш
	rm -f log.csv
	# компилируем
	/usr/bin/g++ -g "./src/main1.cpp" -o "./src/main1"
	/usr/bin/g++ -g "./src/main2.cpp" -o "./src/main2"
	/usr/bin/g++ -g "./src/main3.cpp" -o "./src/main3"
	/usr/bin/g++ -g "./src/main4.cpp" -o "./src/main4"
	
run1:
	# чистим кэш
	rm -f log.csv
	# запускаем что получилось
	./src/main1

run2:
	# чистим кэш
	rm -f log.csv
	# запускаем что получилось
	./src/main2

run3:
	# чистим кэш
	rm -f log.csv
	# запускаем что получилось
	./src/main3

run4:
	# чистим кэш
	rm -f log.csv
	# запускаем что получилось
	./src/main4

