import os, sys, glob, time, pathlib, argparse
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '5'

# Kerasa / TensorFlow
from loss import depth_loss_function
from utils import predict, save_images, load_test_data
from model import create_model
from data import get_nyu_train_test_data, get_unreal_train_test_data, get_eyemodel_train_test_data
from callbacks import get_nyu_callbacks, get_eyemodel_callbacks

from keras.optimizers import Adam
from keras.utils import multi_gpu_model
from keras.utils.vis_utils import plot_model

# Argument Parser
# name または flags - 名前か、あるいはオプション文字列のリスト (例: foo や -f, --foo)。
# action - コマンドラインにこの引数があったときのアクション。
# nargs - 受け取るべきコマンドライン引数の数。
# const - 一部の action と nargs の組み合わせで利用される定数。
# default - コマンドラインに引数がなかった場合に生成される値。
# type - コマンドライン引数が変換されるべき型。
# choices - 引数として許される値のコンテナー。
# required - コマンドラインオプションが省略可能かどうか (オプション引数のみ)。
# help - 引数が何なのかを示す簡潔な説明。
# metavar - 使用法メッセージの中で使われる引数の名前。
# dest - parse_args() が返すオブジェクトに追加される属性名。
parser = argparse.ArgumentParser(description='High Quality Monocular Depth Estimation via Transfer Learning')
parser.add_argument('--data', default='nyu', type=str, help='Training dataset.')
parser.add_argument('--lr', type=float, default=0.0001, help='Learning rate')
parser.add_argument('--bs', type=int, default=4, help='Batch size')
parser.add_argument('--epochs', type=int, default=20, help='Number of epochs')
parser.add_argument('--gpus', type=int, default=1, help='The number of GPUs to use')
parser.add_argument('--gpuids', type=str, default='0', help='IDs of GPUs to use')
parser.add_argument('--mindepth', type=float, default=10.0, help='Minimum of input depths')
parser.add_argument('--maxdepth', type=float, default=1000.0, help='Maximum of input depths')
parser.add_argument('--name', type=str, default='densedepth_nyu', help='A name to attach to the training session')
parser.add_argument('--checkpoint', type=str, default='', help='Start training from an existing model.')
parser.add_argument('--full', dest='full', action='store_true', help='Full training with metrics, checkpoints, and image samples.')

args = parser.parse_args()

# Inform about multi-gpu training
if args.gpus == 1: 
    os.environ['CUDA_VISIBLE_DEVICES'] = args.gpuids
    print('Will use GPU ' + args.gpuids)
else:
    print('Will use ' + str(args.gpus) + ' GPUs.')

# Create the model
model = create_model( existing=args.checkpoint )

# Data loaders
if args.data == 'nyu':
     train_generator, test_generator = get_nyu_train_test_data( args.bs )
if args.data == 'unreal': 
    train_generator, test_generator = get_unreal_train_test_data( args.bs )
if args.data == 'eyemodel': 
    train_generator, test_generator = get_eyemodel_train_test_data( args.bs )

# Training session details
runID = str(int(time.time())) + '-n' + str(len(train_generator)) + '-e' + str(args.epochs) + '-bs' + str(args.bs) + '-lr' + str(args.lr) + '-' + args.name
outputPath = './models/'
runPath = outputPath + runID
pathlib.Path(runPath).mkdir(parents=True, exist_ok=True)
print('Output: ' + runPath)

 # (optional steps)
if True:
    # Keep a copy of this training script and calling arguments
    with open(__file__, 'r', encoding="utf-8") as training_script: training_script_content = training_script.read()
    training_script_content = '#' + str(sys.argv) + '\n' + training_script_content
    with open(runPath+'/'+__file__, 'w') as training_script: training_script.write(training_script_content)

    # Generate model plot
    plot_model(model, to_file=runPath+'/model_plot.svg', show_shapes=True, show_layer_names=True)

    # Save model summary to file
    from contextlib import redirect_stdout
    with open(runPath+'/model_summary.txt', 'w') as f:
        with redirect_stdout(f): model.summary()

# Multi-gpu setup:
basemodel = model
if args.gpus > 1:
     model = multi_gpu_model(model, gpus=args.gpus)

# Optimizer
optimizer = Adam(lr=args.lr, amsgrad=True)

# Compile the model
print('\n\n\n', 'Compiling model..', runID, '\n\n\tGPU ' + (str(args.gpus)+' gpus' if args.gpus > 1 else args.gpuids)
        + '\t\tBatch size [ ' + str(args.bs) + ' ] ' + ' \n\n')
model.compile(loss=depth_loss_function, optimizer=optimizer)

print('Ready for training!\n')

# Callbacks
# 訓練中にモデル内部の状態と統計量を可視化する際に，コールバックを使う
# ここではTensorBoardを用いて訓練とテストの評価値を動的にグラフ化し，可視化する。
callbacks = []
if args.data == 'nyu':
     callbacks = get_nyu_callbacks(model, basemodel, train_generator, test_generator, load_test_data() if args.full else None , runPath)
if args.data == 'unreal':
     callbacks = get_nyu_callbacks(model, basemodel, train_generator, test_generator, load_test_data() if args.full else None , runPath)
if args.data == 'eyemodel':
     callbacks = get_eyemodel_callbacks(model, basemodel, train_generator, test_generator, load_test_data() if args.full else None , runPath, minDepth=args.mindepth, maxDepth=args.maxdepth, batchsize=args.bs)

# Start training
model.fit_generator(train_generator, callbacks=callbacks, validation_data=test_generator, epochs=args.epochs, shuffle=True)

# Save the final trained model:
print('Save the final trained model!\n')
basemodel.save(runPath + '/model.h5')
