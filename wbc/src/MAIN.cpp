#include "model/RobotModel.hpp"
#include <iostream>
using namespace std;
using namespace dynacore;

struct modelStateData{
	dynacore::Vector _q;
	dynacore::Vector _qdot;
};

int main(void) {
	RobotModel* _model = new RobotModel();
	//_model->PrintLinkList();
	struct modelStateData _state = {
		Eigen::VectorXd::Zero(robot::num_q), 
		Eigen::VectorXd::Zero(robot::num_qdot)
	};
	Vect3 RPY, linkPos, linkOri, linkVel, linkAngVel;
	Quaternion qua;
	Matrix Jacfl, cenInertia;
	Matrix A;
	Mat3 World2BodyInRbdl, Body2WorldMyself;
	RPY << 0.0, 0.0, 0.0;
	convert(RPY[2], RPY[1], RPY[0], qua);
	_state._q << 0.2, 0.2, 0.6, qua.x(), qua.y(), qua.z(),
		0, 0, 0,
		0, 0, 0,
		0, 0, 0,
		0, 0, 0,
		qua.w();
	_state._qdot << Eigen::VectorXd::Zero(6),
		0, 0, 0.,
		0, 0, 0,
		0, 0, 0,
		0, 0, 0;
	_model -> UpdateSystem(_state._q, _state._qdot);

	_model -> getWorld2BodyMatrix(World2BodyInRbdl);
	Body2WorldMyself = RotZ_Matrix(RPY[2]) * RotY_Matrix(RPY[1]) * RotX_Matrix(RPY[0]);
	_model -> getPos(1, linkPos);
	_model -> getOri(1, linkOri);
	_model -> getLinearVel(1, linkVel);
	_model -> getCentroidInertia(cenInertia);
    _model -> getMassInertia(A);
    cout << "A rows : " << A.rows() << " A cols :"<< A.cols() << "\n\n";
    cout << A << "\n\n";
	cout << linkPos.transpose() << "\n\n";
	cout << linkOri.transpose() << "\n\n";
	cout << cenInertia << "\n\n";
	cout << World2BodyInRbdl << "\n\n" << Body2WorldMyself.transpose() << "\n\n";

	 dynacore::Matrix JacFl, JacFr, JacRl, JacRr;
	  _model -> getFullJacobian(1, JacFl);
	 cout << JacFl << "\n\n\n";
	 _model -> getFullJacobian(2, JacFr);
	 cout << JacFr << "\n\n\n";
	 _model -> getFullJacobian(3, JacRl);
	 cout << JacRl << "\n\n\n";
	 _model -> getFullJacobian(4, JacRr);
	 cout << JacRr << "\n\n\n";
    dynacore::Matrix  JacBody;
    //_model->getCoMJacobian(JacBody);
    _model->getCentroidJacobian(JacBody);
    cout << JacBody.rows() << " " << JacBody.cols() << "\n\n";
    cout << JacBody.block(0,0,JacBody.rows(),JacBody.cols());
	return 0;
}