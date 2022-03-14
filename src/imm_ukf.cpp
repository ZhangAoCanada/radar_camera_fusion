#include "imm_ukf.h"

void IMM_UKF::IMM_Initialization(Eigen::VectorXd& Z, float time, float velo, float angle){
	//TODO initial x_merge_ interact_pro_
	isinitialized = true;
	
	Eigen::VectorXd x(n_x_);
	x.fill(0.0);
	for(int k=0; k<n_z_; ++k){
		x(k) = Z(k);
	}

	x(n_z_) = velo;
	x(n_z_+1) = angle;
	//初始化状态量
	for(int i=0; i<model_size; ++i){
		model_X_[i] = x;
	}

	for(int i=0; i<model_size; ++i){
		imm_ukf_[i].Initialization(model_X_[i],model_P_[i], time);
	}
}

//输入交互
void IMM_UKF::InputInteract(){
	if(std::isnan(model_pro_(0))) {
		std::abort();
    }
	if(model_pro_.sum() !=0)
		model_pro_ /model_pro_.sum();

	c_.fill(0.0);
	//the jth model
	for(int j=0; j<model_size; ++j){
		model_X_[j] = imm_ukf_[j].Get_state();//上一时刻的结果
		model_P_[j] = imm_ukf_[j].Get_covariance();
		for(int i=0; i<model_size; ++i){
			c_(j) += interact_pro_(i,j)*model_pro_(i); //i转移到j的概率
		}
	}
	for(int j=0; j<model_size; ++j){
		X_hat_[j].fill(0.);
		P_hat_[j].fill(0.);
		for(int i=0; i<model_size; ++i){
			float u =  ((interact_pro_(i,j)*model_pro_(i))/c_(j));
			X_hat_[j] += u * model_X_[i]; //problem
		}
		for(int i=0; i<model_size; ++i){
			float u =  (interact_pro_(i,j)*model_pro_(i))/c_(j);		
			P_hat_[j] += (u * (model_P_[i] + (model_X_[i] - X_hat_[j])*(model_X_[i] - X_hat_[j]).transpose()));
		}
	}		
	
}	

//预测
void IMM_UKF::PredictionZmerge(float time){
	InputInteract();

	std::vector<float> model_ita(model_size);

	S_merge_ = Eigen::MatrixXd(n_z_,n_z_);
	S_merge_.fill(0.0);

	Zpre_merge_ = Eigen::VectorXd(n_z_);
	Zpre_merge_.fill(0.0);

	for(int i=0; i<model_size; ++i){
		imm_ukf_[i].PredictionZ(X_hat_[i], P_hat_[i], time);
		//TODO get zmerge
		Zpre_merge_ += (model_pro_(i) * imm_ukf_[i].Get_PredictionZ());
	}

	for(int i=0; i<model_size; ++i){
		Eigen::MatrixXd S = imm_ukf_[i].Get_S();
		Eigen::VectorXd Zp = imm_ukf_[i].Get_PredictionZ();
		S_merge_ += ( model_pro_(i) * (S + (Zp - Zpre_merge_)*(Zp - Zpre_merge_).transpose()));
	}

}

//模型以及概率更新		
void IMM_UKF::UpdateProbability(std::vector<Eigen::VectorXd>& Z, const Eigen::VectorXd& beta, const float& last_beta){

	std::vector<float> model_ita(model_size);

	for(int i=0; i<model_size; ++i){

		imm_ukf_[i].Update(Z, beta, last_beta);
	
		Eigen::VectorXd Zminus = imm_ukf_[i].Get_Zminus();
		Eigen::MatrixXd S = imm_ukf_[i].Get_S();

        // NOTE: using fabs() instead of abs() for the double type.
        // also NOTE: fabs() is to avoid the negative value of the sqrt.
		float ita = 1/(sqrt(2*pi_)*sqrt(fabs(S.determinant())))*exp(-0.5* Zminus.transpose() * S.inverse() * Zminus);
		model_ita[i] = ita;
	}
	
	float c_temp = 0;	
	for(int i=0; i<model_size; ++i){
		c_temp += model_ita[i] * c_(i);
	}

	for(int i=0; i<model_size; ++i){
		model_pro_(i) = (model_ita[i]*c_(i))/c_temp;
		if(model_pro_(i)<1e-4) model_pro_(i) = 1e-4;
	}

	MixProbability();

}

//模型以及概率更新
void IMM_UKF::UpdateProbability(Eigen::VectorXd& Z){

	std::vector<float> model_ita(model_size);
	for(int i=0; i<model_size; ++i){

		imm_ukf_[i].Update(Z);

		Eigen::VectorXd Zminus = imm_ukf_[i].Get_Zminus();
		Eigen::MatrixXd S = imm_ukf_[i].Get_S();

		float ita = 1/(sqrt(2*pi_)*sqrt(fabs(S.determinant())))*exp(-0.5* Zminus.transpose() * S.inverse() * Zminus);
		model_ita[i] = ita;
	}

	float c_temp = 0;
	for(int i=0; i<model_size; ++i){
		c_temp += model_ita[i] * c_(i);
	}

	for(int i=0; i<model_size; ++i){
		model_pro_(i) = (model_ita[i]*c_(i))/c_temp;
		if(model_pro_(i)<1e-4) model_pro_(i) = 1e-4;
	}

	MixProbability();

}

//输出交互
void IMM_UKF::MixProbability(){
	x_merge_.fill(0.0);
	p_merge_.fill(0.0);
	for(int i=0; i<model_size; ++i){
		model_X_[i] = imm_ukf_[i].Get_state();//当前时刻更新结果
		model_P_[i] = imm_ukf_[i].Get_covariance();
		x_merge_ += model_X_[i] * model_pro_(i);
	}

	for(int i=0; i<model_size; ++i){
		p_merge_ += model_pro_(i) * (model_P_[i] + (model_X_[i] -x_merge_)* (model_X_[i] -x_merge_).transpose());
	}
}


void IMM_UKF::Process(std::vector<Eigen::VectorXd>& Z, const Eigen::VectorXd& beta, const float& last_beta, float& time){
	if(!IsInitialized()){
		IMM_Initialization(Z[0], time,0,0);
	}else{
		PredictionZmerge(time);
		UpdateProbability(Z, beta, last_beta);
	}
}


Eigen::VectorXd IMM_UKF::getMixState(){
	return x_merge_;
}

Eigen::MatrixXd IMM_UKF::getMixCovariance(){

	return p_merge_;
}

Eigen::MatrixXd IMM_UKF::GetS(){
	return S_merge_;
}

Eigen::VectorXd IMM_UKF::GetZpre(){
	return Zpre_merge_;
}
