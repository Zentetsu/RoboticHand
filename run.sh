validate=0
SendPos=0
Inverse=0
Generic=0
UpdateRob=0


while [[ $# -gt 0 ]]; do
	key="$1"

	case $key in
		--SendPos)
		SendPos=1
		shift # past value
		;;
		--Inverse)
		Inverse=1
		shift # past value
		;;
		--Generic)
		Generic=1
		shift # past value
		;;
		--UpdateRob)
		UpdateRob=1
		shift # past value
		;;
		--default)
		shift # past argument
		;;
		*)    # unknown option
		validate=-1
		break
		shift # past argument
		;;
	esac
done

if [[ $validate -eq -1 ]]; then
	echo "ERROR: wrong arguments."
	exit 1
fi

if [[ $SendPos -eq 1 ]]; then
	python "./src/sendPosition.py" &
fi

if [[ $Inverse -eq 1 ]]; then
	python "runSofaScript.py" -c --arm750 --sm & # without hand
	#python "runSofaScript.py" -c --arm750 --sm --hand & # with hand
fi

if [[ $Generic -eq 1 ]]; then
	python "runSofaScript.py" -g -c --arm750 --sm & # without hand
	# python "runSofaScript.py" -g -c --arm750 --sm --hand & # with hand
fi